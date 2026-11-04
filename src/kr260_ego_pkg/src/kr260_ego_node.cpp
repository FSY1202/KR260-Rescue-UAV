// No auto-arm, mode acquisition, legacy trajectory server, or automatic recovery.
#include <kr260_ego_core/mission.hpp>
#include <kr260_ego_core/preparation.hpp>
#include <kr260_ego_interfaces/msg/map_snapshot.hpp>
#include <kr260_ego_interfaces/msg/planner_health.hpp>
#include <kr260_ego_interfaces/msg/person_observation.hpp>
#include <kr260_ego_interfaces/msg/drop_request.hpp>
#include <kr260_ego_interfaces/msg/drop_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <unistd.h>

using namespace kr260_ego;
using namespace std::chrono_literals;
namespace fs=std::filesystem;
static_assert(px4_msgs::msg::VehicleLocalPosition::MESSAGE_VERSION==1,"pinned local position required");
static_assert(px4_msgs::msg::VehicleStatus::MESSAGE_VERSION==3,"pinned status required");
static_assert(px4_msgs::msg::VehicleCommandAck::MESSAGE_VERSION==1,"pinned ack required");
static_assert(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL==2&&px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD==14&&px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND==18,"pinned mode constants required");
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static std::string read(const fs::path& p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
static std::string quote(const std::string& s){std::string o="\"";for(unsigned char c:s){if(c=='"'||c=='\\')o+='\\';if(c>=32)o+=c;}return o+'"';}
static std::string vec(V v){std::ostringstream s;s<<std::setprecision(12)<<'['<<v.x<<','<<v.y<<','<<v.z<<']';return s.str();}
static void save(const fs::path& p,const std::string& s){auto tmp=p.string()+".tmp";{std::ofstream f(tmp);f<<s;f.flush();if(!f)throw std::runtime_error("state write failed");}fs::rename(tmp,p);}
static double json_number(const std::string& s,const std::string& key){auto p=s.find('"'+key+'"');if(p==std::string::npos)return -1e9;p=s.find(':',p);if(p==std::string::npos)return -1e9;return std::strtod(s.c_str()+p+1,nullptr);}
static std::string json_text(const std::string& s,const std::string& key){auto p=s.find('"'+key+'"');if(p==std::string::npos)return {};p=s.find(':',p);if(p==std::string::npos)return {};p=s.find('"',p+1);if(p==std::string::npos)return {};auto end=s.find('"',p+1);return end==std::string::npos?"":s.substr(p+1,end-p-1);}
struct Track {
  uint64_t stamp{};double rx{-1e9};
  bool update(uint64_t t,bool ok,bool& backwards){if(stamp&&t<stamp)backwards=true;if(!t||t<=stamp||!ok)return false;stamp=t;rx=mono();return true;}
  bool fresh(double now,double limit)const{return now>=rx&&now-rx<=limit;}
};
class EgoNode:public rclcpp::Node {
 public:
  EgoNode():Node("kr260_ego_controller"){
    ros_clock_=get_clock();
    Settings s;s.height=declare_parameter("height_m",1.);s.forward=declare_parameter("forward_m",3.);s.left=declare_parameter("left_m",0.);
    s.speed=declare_parameter("speed_mps",.2);s.acceleration=declare_parameter("acceleration_mps2",.2);s.settle=declare_parameter("settle_seconds",2.);
    auto lo=declare_parameter<std::vector<double>>("site_min",{-1.,-2.,.6}),hi=declare_parameter<std::vector<double>>("site_max",{4.,2.,1.4});
    if(lo.size()!=3||hi.size()!=3)throw std::runtime_error("site bounds need 3 coordinates");
    const double floor[3]={-1,-2,.6},ceiling[3]={4,2,1.4};
    for(int i=0;i<3;++i)if(!std::isfinite(lo[i])||!std::isfinite(hi[i])||lo[i]<floor[i]||hi[i]>ceiling[i]||lo[i]>=hi[i])
      throw std::runtime_error("site must be within initial flight envelope");
    s.flight={{lo[0],lo[1],lo[2]},{hi[0],hi[1],hi[2]}};
    const auto route=declare_parameter<std::vector<double>>("route_points_m",std::vector<double>{});
    if(route.size()%3||route.size()>96)throw std::runtime_error("route_points_m must contain at most 32 xyz triples");
    for(size_t i=0;i<route.size();i+=3)s.route_points.push_back({route[i],route[i+1],route[i+2]});
    s.rescue_enabled=declare_parameter("rescue_enabled",true);
    s.rescue_hover=declare_parameter("rescue_hover_seconds",5.);
    s.rescue_wait=declare_parameter("rescue_wait_seconds",15.);
    s.drop_timeout=declare_parameter("drop_timeout_seconds",5.);
    drop_pulse_=declare_parameter("drop_pulse_us",1000);
    drop_duration_=declare_parameter("drop_duration_seconds",1.);
    if(drop_pulse_<500||drop_pulse_>1700||!std::isfinite(drop_duration_)||drop_duration_<.2||drop_duration_>3||s.drop_timeout<drop_duration_+1)
      throw std::runtime_error("drop requires 500..1700us, 0.2..3s and at least 1s completion margin");
    mission_=Mission(s);
    execute_=declare_parameter("execute",false);runtime_=declare_parameter<std::string>("runtime_dir","/run/kr260-ego");run_=declare_parameter<std::string>("run_id","");
    const bool site=declare_parameter("site_confirmed",false),takeoff=declare_parameter("takeoff_clear",false),landing=declare_parameter("landing_clear",false);
    radius_=declare_parameter("airframe_radius_m",0.);half_height_=declare_parameter("airframe_half_height_m",0.);
    declared_=site&&takeoff&&landing&&radius_>0&&radius_<=.7&&half_height_>0&&half_height_<=.5;
    const auto supervisor=read(runtime_/"supervisor.json");
    if(run_.empty()||(execute_&&(!declared_||supervisor.find(run_)==std::string::npos)))throw std::runtime_error("execute requires owned supervisor and measured/declared site");
    if(get_parameter("use_sim_time").as_bool())throw std::runtime_error("use_sim_time forbidden");
    localization_=declare_parameter<std::string>("localization_state_file","/run/kr260-position/state.json");
    if(localization_!="/run/kr260-position/state.json"){
      const char* d=getenv("ROS_DOMAIN_ID"),*l=getenv("ROS_LOCALHOST_ONLY");
      if(!d||std::string(d)=="0"||!l||std::string(l)!="1")throw std::runtime_error("test status requires isolated ROS domain + localhost-only");
    }
    auto qos=rclcpp::SensorDataQoS().keep_last(1);
    pos_sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1",qos,[this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m){
      bool valid=finite(V{m->x,m->y,m->z})&&finite(V{m->vx,m->vy,m->vz})&&std::isfinite(m->heading)&&m->xy_valid&&m->z_valid&&m->v_xy_valid&&m->v_z_valid&&m->heading_good_for_control&&!m->dead_reckoning;
      const std::array<uint8_t,5> r{{m->xy_reset_counter,m->z_reset_counter,m->heading_reset_counter,m->vxy_reset_counter,m->vz_reset_counter}};
      const bool timed=age(m->timestamp,.3)&&age(m->timestamp_sample,.3)&&m->timestamp_sample<=m->timestamp;
      pos_valid_=valid;
      const bool changed=pos_seen_&&r!=resets_;
      if(pt_.update(m->timestamp_sample,valid&&timed,backwards_)){
        if(changed){
          if(preparation_.bootstrap()){
            // No task reference/output exists yet. Ground telemetry must authorize
            // a fresh wait before this uncommitted pairing can be used.
            preparation_reset_pending_=true;poses_.clear();alignment_=Alignment{};align_count_=0;
          }else reset_=true;
        }
        resets_=r;pos_seen_=true;pos_=*m;poses_.push_back(*m);if(poses_.size()>64)poses_.pop_front();
      }else if(changed&&!preparation_.bootstrap()){
        reset_=true;resets_=r;pos_seen_=true;
      }
    });
    status_sub_=create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v3",qos,[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m){
      if(!status_seen_&&m->arming_state==m->ARMING_STATE_ARMED)first_armed_=true;status_seen_=true;
      if(m->arming_state==m->ARMING_STATE_ARMED)ever_armed_=true;
      if(st_.update(m->timestamp,age(m->timestamp,2),backwards_))status_=*m;
    });
    land_sub_=create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",qos,[this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr m){if(lt_.update(m->timestamp,age(m->timestamp,2),backwards_))land_=*m;});
    fail_sub_=create_generic_subscription("/fmu/out/failsafe_flags","px4_msgs/msg/FailsafeFlags",qos,[this](std::shared_ptr<rclcpp::SerializedMessage> raw){
      ++flag_raw_;try{
        if(raw->size()!=96)throw std::runtime_error("FailsafeFlags CDR incompatible with pinned 96-byte firmware");
        px4_msgs::msg::FailsafeFlags m;rclcpp::Serialization<px4_msgs::msg::FailsafeFlags> codec;codec.deserialize_message(raw.get(),&m);
        if(ft_.update(m.timestamp,age(m.timestamp,2),backwards_))flags_=m;
      }catch(const std::exception& e){++flag_errors_;event(e.what());}
    });
    vo_sub_=create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",qos,[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr m){
      bool ok=age(m->timestamp_sample,.2)&&age(m->timestamp,.2)&&m->timestamp_sample<=m->timestamp&&m->pose_frame==m->POSE_FRAME_NED;
      for(auto x:m->position)ok=ok&&std::isfinite(x);
      double qnorm=0;for(auto x:m->q){ok=ok&&std::isfinite(x);qnorm+=x*x;}
      ok=ok&&std::abs(qnorm-1)<.01;vt_.update(m->timestamp_sample,ok,backwards_);
    });
    lio_sub_=create_subscription<nav_msgs::msg::Odometry>("/Odometry",qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){align(m);});
    map_sub_=create_subscription<kr260_ego_interfaces::msg::MapSnapshot>("/kr260/ego/map",rclcpp::QoS(1),[this](kr260_ego_interfaces::msg::MapSnapshot::ConstSharedPtr m){
      if(m->run_id!=run_||m->header.frame_id!="camera_init")return;
      const double stamp=rclcpp::Time(m->source_stamp).seconds();if(!age(uint64_t(stamp*1e6),.3)||m->revision<=map_.revision)return;
      try{map_.load_snapshot({m->origin.x,m->origin.y,m->origin.z},m->resolution,m->size_x,m->size_y,m->size_z,m->cells,stamp,m->revision);map_rx_=mono();}
      catch(const std::exception& e){map_error_=e.what();event("map rejected: "+map_error_);}
    });
    health_sub_=create_subscription<kr260_ego_interfaces::msg::PlannerHealth>("/kr260/ego/planner_health",rclcpp::QoS(1),[this](kr260_ego_interfaces::msg::PlannerHealth::ConstSharedPtr m){
      if(m->run_id!=run_||m->header.frame_id!="camera_init"||!std::isfinite(m->planning_ms)||m->planning_ms<0)return;
      const auto t=uint64_t(rclcpp::Time(m->header.stamp).nanoseconds()/1000);
      if(ht_.update(t,age(t,.35),backwards_))health_=*m;
    });
    spline_sub_=create_subscription<traj_utils::msg::Bspline>("/kr260/ego/bspline",rclcpp::QoS(1),[this](traj_utils::msg::Bspline::ConstSharedPtr m){accept(m);});
    person_sub_=create_subscription<kr260_ego_interfaces::msg::PersonObservation>("/kr260/rescue/person",rclcpp::QoS(1),[this](kr260_ego_interfaces::msg::PersonObservation::ConstSharedPtr m){
      // Only observations obtained during the current cruise can authorize rescue.
      if(!execute_||!mission_.triggered||(mission_.state!="FOLLOW_EGO"&&mission_.state!="GOAL_SETTLE")){
        person_gate_=PersonConfirmation{};person_until_=0;return;
      }
      PersonObservation o{m->detector_session,m->camera_session,m->result_seq,m->frame_seq,m->source_monotonic_ns,m->person_detected,m->confidence};
      const bool confirmed=person_gate_.observe(o,uint64_t(mono()*1e9));
      if(confirmed)person_until_=o.source_ns/1e9+2.;
      else if(person_gate_.count<2)person_until_=0;
    });
    drop_feedback_sub_=create_subscription<kr260_ego_interfaces::msg::DropFeedback>("/kr260/rescue/drop_feedback",rclcpp::QoS(10),[this](kr260_ego_interfaces::msg::DropFeedback::ConstSharedPtr m){
      if(m->run_id!=run_||!age(uint64_t(rclcpp::Time(m->header.stamp).nanoseconds()/1000),.7))return;
      if(m->state==m->READY&&m->request_id==0&&m->outputs_disabled)actuator_ready_at_=mono();
      if(pending_drop_&&m->request_id==pending_drop_){
        if(m->state==m->COMPLETE&&m->outputs_disabled)drop_result_=DropResult::COMPLETE;
        else if(m->state==m->FAILED)drop_result_=DropResult::FAILED;
      }
    });
    drop_pub_=create_publisher<kr260_ego_interfaces::msg::DropRequest>("/kr260/rescue/drop_request",rclcpp::QoS(10));
    ack_sub_=create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack_v1",qos,[this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr m){if(m->command==21&&m->target_component==191)event("NAV_LAND ack="+std::to_string(m->result));});
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/kr260/ego/odometry",rclcpp::QoS(1));goal_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/kr260/ego/goal",rclcpp::QoS(1));
    if(!execute_)preview_odom_=create_publisher<nav_msgs::msg::Odometry>("/kr260/ego/preview_odometry",rclcpp::QoS(1));
    columns_pub_=create_publisher<geometry_msgs::msg::PoseArray>("/kr260/ego/clear_columns",rclcpp::QoS(1).transient_local());
    diag_=create_publisher<std_msgs::msg::String>("/kr260/ego/status",10);events_=create_publisher<std_msgs::msg::String>("/kr260/ego/events",10);
    if(execute_){hb_=create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",10);sp_=create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);cmd_=create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",10);}
    start_=mission_start_=last_tick_=mono();io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    io_timer_=create_wall_timer(200ms,[this]{io();},io_group_);timer_=create_wall_timer(50ms,[this]{tick();});
    event(execute_?"EGO preparation: Position arm then ground Offboard; NOT flight clearance":"PREVIEW: no flight input publishers");
  }
 private:
  uint64_t now_us()const{return ros_clock_->now().nanoseconds()/1000;}
  bool age(uint64_t stamp,double max)const{const double a=(int64_t(now_us())-int64_t(stamp))/1e6;return stamp>0&&a>=-.01&&a<=max;}
  V actual()const{return {pos_.x,pos_.y,pos_.z};}V velocity()const{return {pos_.vx,pos_.vy,pos_.vz};}
  void event(const std::string& s){std::lock_guard<std::mutex> l(io_mutex_);pending_.push_back("{\"run_id\":"+quote(run_)+",\"timestamp_us\":"+std::to_string(now_us())+",\"event\":"+quote(s)+"}");if(pending_.size()>100)pending_.pop_front();}
  bool bounds(V map)const{return mission_.in_bounds(alignment_.to_ned(map));}
  void align(nav_msgs::msg::Odometry::ConstSharedPtr m){
    const uint64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds()/1000;
    const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;
    const double qnorm=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    if(m->header.frame_id!="camera_init"||!finite(V{p.x,p.y,p.z})||!std::isfinite(qnorm)||std::abs(qnorm-1)>.01||!lio_track_.update(stamp,age(stamp,.2),backwards_))return;
    if(poses_.empty())return;
    auto match=std::min_element(poses_.begin(),poses_.end(),[&](const auto& a,const auto& b){return std::abs(int64_t(a.timestamp_sample)-int64_t(stamp))<std::abs(int64_t(b.timestamp_sample)-int64_t(stamp));});
    const double delta=(int64_t(match->timestamp_sample)-int64_t(stamp))/1e6;if(std::abs(delta)>.05)return;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    if(alignment_.frozen){
      if(norm(alignment_.to_ned({p.x,p.y,p.z})-V{match->x,match->y,match->z})>.25||std::abs(angle(alignment_.yaw_to_ned(yaw)-match->heading))>5*pi/180)reset_=true;
    }else if(!status_seen_||status_.arming_state!=status_.ARMING_STATE_ARMED){
      Alignment candidate;candidate.observe({p.x,p.y,p.z},{match->x,match->y,match->z},yaw,match->heading,delta);
      if(alignment_.valid&&(norm(candidate.translation-alignment_.translation)>.1||std::abs(angle(candidate.delta-alignment_.delta))>.05))align_count_=0;
      alignment_=candidate;++align_count_;
    }
  }
  void accept(traj_utils::msg::Bspline::ConstSharedPtr m){
    if((!alignment_.frozen&&execute_)||!goal_sent_||m->traj_id<=last_id_)return;
    try{
      if(m->kr260_run_id!=run_||m->kr260_goal_stamp.sec!=goal_stamp_.sec||m->kr260_goal_stamp.nanosec!=goal_stamp_.nanosec)
        throw std::runtime_error("trajectory belongs to another run or route goal");
      if(!planner_sources_ok_||!health_.map_valid||health_.emergency||!ht_.fresh(mono(),.35)||!map_.fresh(now_us()/1e6))throw std::runtime_error("planner/map not independently healthy");
      const double start=rclcpp::Time(m->start_time).seconds(),now=now_us()/1e6;
      if(start<goal_wall_-.05||now-start>.2||start-now>.05)throw std::runtime_error("stale/future/origin-mismatched trajectory");
      std::vector<V> p;for(const auto& v:m->pos_pts)p.push_back({v.x,v.y,v.z});
      auto candidate=Spline::make(m->order,p,m->knots,start,m->traj_id,mission_.settings.speed,mission_.settings.acceleration);
      const V reference=execute_?actual():actual()+V{0,0,-mission_.settings.height},vel=execute_?velocity():V{};
      if(norm(alignment_.to_ned(candidate.sample(now))-reference)>.15||norm(alignment_.vector_to_ned(candidate.speed(now))-vel)>.1)throw std::runtime_error("trajectory discontinuity");
      if(!safe_curve(candidate,map_,[this](V p){return bounds(p);},std::max(start,now),start+candidate.duration()))throw std::runtime_error("trajectory collision/unknown/outside site");
      curve_=std::move(candidate);last_id_=m->traj_id;trajectory_error_.clear();
    }catch(const std::exception& e){++rejected_;trajectory_error_=e.what();event("trajectory rejected: "+trajectory_error_);}
  }
  bool brake_clear()const{
    if(!alignment_.frozen||!map_.revision||now_us()/1e6-map_.source_time>2||!finite(velocity())||norm(velocity())>.4)return false;
    const double speed=norm(velocity()),duration=speed/mission_.settings.acceleration;V last=alignment_.to_map(actual());
    for(double t=0;t<=duration+.025;t+=.025){const double tt=std::min(t,duration);V p=alignment_.to_map(actual()+velocity()*(speed>1e-8?tt-.5*mission_.settings.acceleration*tt*tt/speed:0));if(!bounds(p)||!map_.line_free(last,p))return false;last=p;}return true;
  }
  void publish_odom(bool task_reference){
    if(!alignment_.valid||!pt_.fresh(mono(),.3))return;
    nav_msgs::msg::Odometry o;o.header.frame_id="camera_init";o.child_frame_id="kr260_ego_body_flu";o.header.stamp=rclcpp::Time(int64_t(pt_.stamp)*1000);
    const V p=alignment_.to_map(actual()),v=alignment_.vector_to_map(velocity());const double yaw=angle(alignment_.delta-pos_.heading);
    o.pose.pose.position.x=p.x;o.pose.pose.position.y=p.y;o.pose.pose.position.z=p.z;o.pose.pose.orientation.z=std::sin(yaw/2);o.pose.pose.orientation.w=std::cos(yaw/2);
    // Planning child is intentionally yaw-level FLU; rotate world velocity into that child.
    const V body=rz(v,-yaw);o.twist.twist.linear.x=body.x;o.twist.twist.linear.y=body.y;o.twist.twist.linear.z=body.z;odom_pub_->publish(o);
    // A separately named, explicitly synthetic hover start is ONLY for ground
    // preview of the post-climb path. Actual planning odometry is never rewritten.
    if(preview_odom_){
      // Warm the preview planner with measured ground observations first. The
      // virtual post-climb reference belongs only to the committed task.
      if(task_reference){o.pose.pose.position.z+=mission_.settings.height;o.child_frame_id="kr260_ego_virtual_body_flu";o.twist.twist.linear={};}
      preview_odom_->publish(o);
    }
  }
  void send_goal(V ned){
    geometry_msgs::msg::PoseStamped m;m.header.frame_id="camera_init";m.header.stamp=get_clock()->now();const V p=alignment_.to_map(ned);
    curve_=Spline{};goal_stamp_=m.header.stamp;
    m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=p.z;m.pose.orientation.w=1;goal_pub_->publish(m);
    goal_sent_=true;goal_wall_=now_us()/1e6;event("submitted this invocation's map-frame goal");
  }
  void drop_request(unsigned id,bool cancel){
    if(!execute_)return;
    kr260_ego_interfaces::msg::DropRequest m;m.header.stamp=get_clock()->now();m.run_id=run_;m.request_id=id;
    m.cancel=cancel;m.pulse_us=drop_pulse_;m.duration_s=drop_duration_;m.valid_for_s=1.;drop_pub_->publish(m);
    event(cancel?"drop cancelled/released":"one bounded servo drop requested");
  }
  void columns(V home,double yaw){
    geometry_msgs::msg::PoseArray a;a.header.frame_id="camera_init";a.header.stamp=get_clock()->now();
    const double map_yaw=angle(alignment_.delta-yaw);
    const V final=mission_.settings.goals().back();
    for(V p:{home,home+rz({final.x,-final.y,0},yaw)}){
      geometry_msgs::msg::Pose x;V m=alignment_.to_map(p);x.position.x=m.x;x.position.y=m.y;x.position.z=m.z;
      x.orientation.z=std::sin(map_yaw/2);x.orientation.w=std::cos(map_yaw/2);a.poses.push_back(x);
    }columns_pub_->publish(a);
  }
  void io(){
    try{
      const auto s=read(localization_);const double stamp=json_number(s,"updated_monotonic"),a=mono()-stamp;
      const auto run=json_text(s,"run_id");
      {std::lock_guard<std::mutex> l(io_mutex_);localization_state_={json_text(s,"state")=="READY"&&a>=0&&a<15&&!run.empty(),stamp,run};}
      bool conflict=false;
      for(const char* topic:{"/fmu/in/trajectory_setpoint","/fmu/in/offboard_control_mode","/fmu/in/vehicle_command"}){
        const auto pubs=get_publishers_info_by_topic(topic);if(pubs.size()>size_t(execute_?1:0))conflict=true;
        for(const auto& p:pubs)if(p.node_name()!=get_name())conflict=true;
      }conflict_=conflict;graph_checked_=true;
      flag_publisher_=!get_publishers_info_by_topic("/fmu/out/failsafe_flags").empty();
      bool readers=true;
      for(const char* topic:{"/fmu/in/trajectory_setpoint","/fmu/in/offboard_control_mode","/fmu/in/vehicle_command"}){
        size_t external=0;
        for(const auto& r:get_subscriptions_info_by_topic(topic))
          if(r.node_name()!="kr260_ego_recorder"&&r.node_name()!="kr260_mission_recorder"&&r.node_name()!=get_name())++external;
        if(external==0)readers=false;
      }dds_matches_=readers;
      bool sources=true;
      for(const char* topic:{"/kr260/ego/bspline","/kr260/ego/map","/kr260/ego/planner_health"}){
        const auto pubs=get_publishers_info_by_topic(topic);if(pubs.size()!=1)sources=false;for(const auto& p:pubs)if(p.node_name()!="kr260_ego_planner")sources=false;
      }
      const auto goals=get_publishers_info_by_topic("/kr260/ego/goal");
      if(goals.size()!=1||goals.front().node_name()!=get_name())sources=false;
      planner_sources_ok_=sources;
      const double when=json_number("{\"time\":"+read(runtime_/"recorder.ready")+"}","time");recorder_ready_=mono()-when>=0&&mono()-when<.7;
      request_stop_=fs::exists(runtime_/"request.stop");request_land_=fs::exists(runtime_/"request.land");
    }catch(const std::exception& e){{std::lock_guard<std::mutex> l(io_mutex_);localization_state_.ready=false;}RCLCPP_ERROR(get_logger(),"health metadata unavailable: %s",e.what());}
    try{
      std::string snapshot;std::deque<std::string> pending;{std::lock_guard<std::mutex> l(io_mutex_);snapshot=snapshot_;pending.swap(pending_);}
      if(!snapshot.empty()){std_msgs::msg::String m;m.data=snapshot;diag_->publish(m);if(execute_||fs::is_directory(runtime_))save(runtime_/"controller.json",snapshot);}
      for(const auto& e:pending){std_msgs::msg::String m;m.data=e;events_->publish(m);RCLCPP_INFO(get_logger(),"%s",e.c_str());}
      if(stop_approved_){save(runtime_/"stop.approved",run_);rclcpp::shutdown();}
    }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"diagnostic/log IO failed; navigation/control retained: %s",e.what());}
  }
  void tick(){
    LocalizationState localization;
    {std::lock_guard<std::mutex> l(io_mutex_);localization=localization_state_;}
    const double before=mono(),wall=now_us()/1e6,now=mono(),dt=now-last_tick_;last_tick_=now;max_gap_=std::max(max_gap_,dt);
    const double offset=wall-(before+now)*.5;if(now-before<.005){if(clock_seen_&&std::abs(offset-clock_offset_)>.05)clock_fault_=true;clock_offset_=offset;clock_seen_=true;}
    if(streaming_&&dt>.2)clock_fault_=true;
    Input in;in.now=now-mission_start_;in.wall=wall;in.seen=status_seen_;in.fresh=st_.fresh(now,2);in.land_fresh=lt_.fresh(now,2);in.landed=land_.landed;
    in.armed=status_.arming_state==status_.ARMING_STATE_ARMED;in.mode=status_.nav_state;in.position=actual();in.velocity=velocity();in.yaw=pos_.heading;
    in.recording=recorder_ready_;in.signal=ft_.fresh(now,2)&&!flags_.offboard_control_signal_lost;in.clock_fault=clock_fault_;in.reset=reset_;in.backwards=backwards_;in.navigation=true;
    in.person_confirmed=now<=person_until_;
    in.actuator_ready=now>=actuator_ready_at_&&now-actuator_ready_at_<.7;
    in.drop_id=pending_drop_;in.drop_result=drop_result_;
    auto check=[&](bool ok,const std::string& why){if(!ok&&in.navigation){in.navigation=false;in.problem=why;}};
    check(in.fresh&&age(st_.stamp,2),"PX4 status stale");check(in.land_fresh&&age(lt_.stamp,2),"land state stale");check(pos_valid_&&pt_.fresh(now,.3)&&age(pt_.stamp,.3),"PX4 position/velocity/heading invalid/stale");
    check(flag_errors_==0,"PX4 failsafe CDR incompatible with pinned 96-byte schema");
    check(flag_raw_>0,flag_publisher_?"failsafe publisher exists but no data":"PX4 failsafe publisher missing");
    check(ft_.fresh(now,2)&&age(ft_.stamp,2),"failsafe timestamp stale/nonadvancing/invalid");
#define FLAG(name) check(!flags_.name,"PX4 flag: " #name)
    FLAG(angular_velocity_invalid);FLAG(attitude_invalid);FLAG(local_altitude_invalid);FLAG(local_position_invalid);FLAG(local_velocity_invalid);
    FLAG(manual_control_signal_lost);FLAG(battery_unhealthy);FLAG(battery_low_remaining_time);FLAG(fd_critical_failure);FLAG(fd_esc_arming_failure);FLAG(fd_imbalanced_prop);FLAG(fd_motor_failure);FLAG(geofence_breached);
#undef FLAG
    check(!status_.failsafe,"PX4 failsafe");check(status_.system_id&&status_.component_id,"PX4 addressing missing");if(!in.armed&&in.mode==2)check(status_.pre_flight_checks_pass,"PX4 preflight checks not passed");
    if(mission_.active())check(in.signal,"PX4 Offboard signal lost");
    check(vt_.fresh(now,.35)&&age(vt_.stamp,.2),"bridge stale/expired");check(localization.ready&&now>=localization.stamp&&now-localization.stamp<15,"localization not READY");check(!conflict_,"foreign Offboard publisher");
    check(!execute_||dds_matches_,"PX4 Offboard DDS readers missing (recorders excluded)");
    check(lio_track_.fresh(now,.3)&&align_count_>=5,"alignment missing/stale");check(!first_armed_,"already armed at startup");check(!clock_fault_&&!backwards_&&!reset_,"clock/frame/time reset");
    const bool ground=in.fresh&&in.land_fresh&&age(st_.stamp,2)&&age(lt_.stamp,2)&&in.landed&&!in.armed&&in.mode==2;
    if(preparation_.bootstrap()&&preparation_reset_pending_&&ground&&!ever_armed_){
      preparation_.note_reset(now);preparation_reset_pending_=false;
      poses_.clear();alignment_=Alignment{};align_count_=0;
      check(false,"rebuilding uncommitted alignment after ground PX4 reset");
    }
    const auto map_difference=health_.map_revision>map_.revision?health_.map_revision-map_.revision:map_.revision-health_.map_revision;
    in.planning=planner_sources_ok_&&map_.fresh(wall)&&now-map_rx_<=.3&&ht_.fresh(now,.35)&&health_.map_valid&&health_.have_odometry&&!health_.emergency&&health_.map_revision>0&&map_difference<=1;
    // First-data planner/map initialization must happen before commitment.
    // Observations may warm the planner; home, columns, goals and control may not.
    if(preparation_.bootstrap()&&in.navigation&&ground&&!ever_armed_&&!preparation_reset_pending_)publish_odom(false);
    const bool was_prepared=preparation_.committed();
    preparation_.step(now,in.navigation&&in.planning&&recorder_ready_&&!preparation_reset_pending_,ground,ever_armed_,
      clock_fault_||backwards_||reset_||flag_errors_>0||(graph_checked_&&conflict_),
      localization.run,localization.stamp,in.problem.empty()?(in.planning?"waiting for fresh navigation and recording":"waiting for initialized fresh EGO map/planner"):in.problem);
    const bool prepared=preparation_.committed();
    if(prepared&&!was_prepared){
      mission_start_=now;in.now=0;event("startup baseline committed after stable ground navigation; runtime guards now latched");
    }
    if(!in.planning&&in.problem.empty())in.problem="map/planner stale, unknown, emergency or source conflict";
    if(prepared&&in.navigation)publish_odom(true);
    if(prepared&&in.navigation&&declared_&&!columns_sent_&&alignment_.valid&&align_count_>=5&&!in.armed&&in.land_fresh&&in.landed){
      column_home_=actual();column_yaw_=in.yaw;columns(column_home_,column_yaw_);columns_sent_=true;
    }
    if(execute_&&columns_sent_&&!alignment_.frozen&&(norm(actual()-column_home_)>.05||std::abs(angle(in.yaw-column_yaw_))>2*pi/180))check(false,"moved outside declared takeoff reference; reprepare");
    if(execute_&&in.planning&&alignment_.valid&&columns_sent_&&!alignment_.frozen){
      const V final=mission_.settings.goals().back();
      const V top=actual()+V{0,0,-mission_.settings.height},goal=actual()+rz({final.x,-final.y,-final.z},in.yaw);
      // Below cruise_min, the explicitly confirmed vertical sites include ground
      // contact. Do not mistake the supporting floor's inflated cells for a wall.
      const double lower=mission_.settings.flight.lower.z;
      in.planning=map_.line_free(alignment_.to_map(actual()+V{0,0,-lower}),alignment_.to_map(top))&&
        map_.line_free(alignment_.to_map(goal),alignment_.to_map(goal+V{0,0,mission_.settings.height-lower}));
      if(!in.planning)in.problem="confirmed vertical site's cruise segment is occupied/unknown";
    }
    if(execute_&&in.planning&&mission_.state=="CLIMB"){
      const double h=std::max(mission_.settings.flight.lower.z,mission_.home.z-actual().z);
      in.planning=map_.line_free(alignment_.to_map(mission_.home+V{0,0,-h}),
                                alignment_.to_map(mission_.home+V{0,0,-mission_.settings.height}));
    }
    const bool route=curve_.valid&&map_.fresh(wall)&&safe_curve(curve_,map_,[this](V p){return bounds(p);},std::max(wall,curve_.start),curve_.start+curve_.duration());
    const bool braking=brake_clear();in.land_requested=request_land_&&mission_.active()&&in.fresh&&in.mode==14;Output out;
    if(request_stop_){
      if(in.fresh&&in.land_fresh&&in.landed&&!in.armed){
        if(!mission_.terminal()&&execute_)mission_.block("ground stop approved");
        stop_approved_=true;
      }else event("stop refused: require fresh landed/disarmed telemetry");
    }
    if(prepared&&execute_&&!stop_approved_){
      if(conflict_&&mission_.active()){mission_.state="TAKEN_OVER";mission_.reason="foreign Offboard publisher; relinquishing without Land/mode command";}
      out=mission_.step(in,curve_.valid?&curve_:nullptr,alignment_,&map_,route,braking);
    }
    else if(prepared&&!execute_){mission_.state="PREVIEW";mission_.reason="virtual post-climb hover preview; no flight input publishers";
      // Preview is grounded: publish a goal for visualization but never control the aircraft.
      if(in.navigation&&in.planning&&!goal_sent_){mission_.home=actual();mission_.yaw=in.yaw;
        const V first=mission_.settings.goals().front();send_goal(actual()+rz({first.x,-first.y,-first.z},in.yaw));}}
    if(execute_&&mission_.state=="WAIT_OFFBOARD_ON_GROUND"&&!alignment_.frozen){alignment_.freeze();columns(mission_.home,mission_.yaw);event("map-to-NED transform frozen at Position arming");}
    if(out.goal)send_goal(mission_.goal);
    if(out.drop){pending_drop_=out.drop_id;drop_result_=DropResult::NONE;drop_request(pending_drop_,false);}
    if(pending_drop_&&mission_.state!="RESCUE_DROP"){
      drop_request(pending_drop_,true);pending_drop_=0;drop_result_=DropResult::NONE;
    }
    const std::string state=prepared?mission_.state:preparation_.state,reason=prepared?mission_.reason:preparation_.reason;
    if(state!=previous_state_){event(state+": "+reason);previous_state_=state;}
    streaming_=execute_&&out.stream&&!stop_approved_;
    if(streaming_){px4_msgs::msg::TrajectorySetpoint sp{};sp.timestamp=now_us();sp.position={float(out.position.x),float(out.position.y),float(out.position.z)};
      sp.velocity.fill(NAN);sp.acceleration.fill(NAN);sp.jerk.fill(NAN);sp.yaw=out.yaw;sp.yawspeed=NAN;sp_->publish(sp);px4_msgs::msg::OffboardControlMode hb{};hb.timestamp=sp.timestamp;hb.position=true;hb_->publish(hb);}
    if(execute_&&out.land){px4_msgs::msg::VehicleCommand c{};c.timestamp=now_us();c.command=c.VEHICLE_CMD_NAV_LAND;c.param4=c.param5=c.param6=c.param7=NAN;
      c.target_system=status_.system_id;c.target_component=status_.component_id;c.source_system=255;c.source_component=191;c.from_external=true;cmd_->publish(c);event("bounded NAV_LAND request; no arm/disarm command");}
    if(now-last_report_>=.2){std::ostringstream s;s<<std::setprecision(15)<<std::boolalpha;
      const bool safe=in.fresh&&in.land_fresh&&in.landed&&!in.armed;
      s<<"{\"run_id\":"<<quote(run_)<<",\"state\":"<<quote(state)<<",\"reason\":"<<quote(reason)<<",\"updated_monotonic\":"<<now<<",\"timestamp_us\":"<<now_us()<<",\"pid\":"<<getpid()<<",\"execute\":"<<execute_<<",\"flight_clearance\":false"
        <<",\"preparation_committed\":"<<prepared<<",\"preparation_state\":"<<quote(preparation_.state)<<",\"preparation_resets\":"<<preparation_.reset_count()
        <<",\"healthy\":"<<(prepared&&in.navigation)<<",\"health_reason\":"<<quote(prepared?in.problem:preparation_.reason)<<",\"planning_healthy\":"<<(prepared&&in.planning)<<",\"safe_to_stop\":"<<safe<<",\"armed\":"<<in.armed<<",\"landed\":"<<in.landed<<",\"nav_state\":"<<in.mode
        <<",\"triggered\":"<<mission_.triggered<<",\"streaming\":"<<streaming_<<",\"alignment_frozen\":"<<alignment_.frozen<<",\"map_revision\":"<<map_.revision<<",\"map_age_ms\":"<<(map_.revision?(wall-map_.source_time)*1000:-1)
        <<",\"preview_virtual_hover\":"<<!execute_
        <<",\"route_index\":"<<mission_.route_index<<",\"route_count\":"<<mission_.settings.goals().size()
        <<",\"rescue_enabled\":"<<mission_.settings.rescue_enabled<<",\"rescue_attempted\":"<<mission_.rescue_attempted<<",\"rescue_complete\":"<<mission_.rescue_complete
        <<",\"drop_request_id\":"<<pending_drop_<<",\"vision_confirmation_count\":"<<person_gate_.count<<",\"actuator_ready\":"<<in.actuator_ready
        <<",\"goal_stamp_sec\":"<<goal_stamp_.sec<<",\"goal_stamp_nanosec\":"<<goal_stamp_.nanosec
        <<",\"planner_ms\":"<<health_.planning_ms<<",\"trajectory_id\":"<<last_id_<<",\"trajectory_rejected\":"<<rejected_<<",\"trajectory_error\":"<<quote(trajectory_error_)<<",\"trajectory_remaining_s\":"<<(curve_.valid?std::max(0.,curve_.start+curve_.duration()-wall):0)
        <<",\"actual_position_ned\":"<<vec(actual())<<",\"actual_velocity_ned\":"<<vec(velocity())<<",\"target_position_ned\":"<<vec(out.position)<<",\"planned_velocity_ned\":"<<vec(out.velocity)<<",\"tracking_error_m\":"<<(mission_.triggered?norm(out.position-actual()):0)
        <<",\"home_ned\":"<<vec(mission_.home)<<",\"alignment_translation\":"<<vec(alignment_.translation)<<",\"alignment_delta_rad\":"<<alignment_.delta
        <<",\"relative_height_m\":"<<(mission_.triggered?mission_.home.z-pos_.z:0)
        <<",\"max_control_gap_s\":"<<max_gap_<<",\"dds_offboard_matched\":"<<bool(dds_matches_)<<",\"recording_ready\":"<<bool(recorder_ready_)<<",\"flags_decode_errors\":"<<flag_errors_<<"}";
      {std::lock_guard<std::mutex> l(io_mutex_);snapshot_=s.str();}last_report_=now;}
  }
  struct LocalizationState {bool ready{};double stamp{};std::string run;};
  Mission mission_;PreparationGate preparation_;Alignment alignment_;VoxelMap map_;Spline curve_;LocalizationState localization_state_;
  PersonConfirmation person_gate_;double person_until_{},actuator_ready_at_{-1e9},drop_duration_{1.};
  unsigned pending_drop_{};int drop_pulse_{1000};DropResult drop_result_{DropResult::NONE};
  rclcpp::Subscription<kr260_ego_interfaces::msg::PersonObservation>::SharedPtr person_sub_;
  rclcpp::Subscription<kr260_ego_interfaces::msg::DropFeedback>::SharedPtr drop_feedback_sub_;
  rclcpp::Publisher<kr260_ego_interfaces::msg::DropRequest>::SharedPtr drop_pub_;
  rclcpp::Clock::SharedPtr ros_clock_;
  bool execute_{},declared_{},pos_seen_{},pos_valid_{},status_seen_{},first_armed_{},ever_armed_{},preparation_reset_pending_{},reset_{},backwards_{},clock_fault_{},clock_seen_{},streaming_{},goal_sent_{},columns_sent_{};
  int align_count_{};int64_t last_id_{-1};size_t rejected_{},flag_raw_{},flag_errors_{};
  double start_{},mission_start_{},last_tick_{},last_report_{},max_gap_{},clock_offset_{},map_rx_{-1e9},goal_wall_{},radius_{},half_height_{},column_yaw_{};V column_home_{};
  Track pt_,st_,lt_,ft_,vt_,ht_,lio_track_;std::array<uint8_t,5> resets_{};
  builtin_interfaces::msg::Time goal_stamp_;
  px4_msgs::msg::VehicleLocalPosition pos_;px4_msgs::msg::VehicleStatus status_;px4_msgs::msg::VehicleLandDetected land_;px4_msgs::msg::FailsafeFlags flags_;
  kr260_ego_interfaces::msg::PlannerHealth health_;std::deque<px4_msgs::msg::VehicleLocalPosition> poses_;
  fs::path runtime_,localization_;std::string run_,previous_state_,snapshot_,map_error_,trajectory_error_;std::mutex io_mutex_;std::deque<std::string> pending_;
  std::atomic<bool> conflict_{true},graph_checked_{false},recorder_ready_{false},planner_sources_ok_{false},dds_matches_{false},flag_publisher_{false},request_stop_{false},request_land_{false},stop_approved_{false};
  rclcpp::CallbackGroup::SharedPtr io_group_;rclcpp::TimerBase::SharedPtr timer_,io_timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::GenericSubscription::SharedPtr fail_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<kr260_ego_interfaces::msg::MapSnapshot>::SharedPtr map_sub_;
  rclcpp::Subscription<kr260_ego_interfaces::msg::PlannerHealth>::SharedPtr health_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_,preview_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr columns_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_,events_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr hb_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<EgoNode>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);executor.add_node(node);executor.spin();}catch(const std::exception& e){std::cerr<<"EGO FATAL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
