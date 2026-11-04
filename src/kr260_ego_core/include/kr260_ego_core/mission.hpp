#pragma once
#include "voxel_map.hpp"
#include "rescue.hpp"

namespace kr260_ego {
struct Input {
  double now{},wall{};V position{},velocity{};double yaw{};
  bool seen{},fresh{},land_fresh{},landed{},armed{},navigation{},planning{},recording{},
       signal{},clock_fault{},reset{},backwards{},stop_requested{},land_requested{};
  int mode{};std::string problem;
  bool person_confirmed{},actuator_ready{};
  unsigned drop_id{};DropResult drop_result{DropResult::NONE};
};
struct Output {bool stream{},land{},goal{},drop{};unsigned drop_id{};V position{},velocity{},acceleration{};double yaw{};};
struct Settings {
  double height{1},forward{3},left{},speed{.2},acceleration{.2},settle{2};
  bool rescue_enabled{false};double rescue_hover{5},rescue_wait{15},drop_timeout{5};
  Bounds flight;
  // Optional takeoff-relative FORWARD / LEFT / UP cruise goals, not map ENU.
  // Empty retains the legacy single forward/left/height target.
  std::vector<V> route_points;
  std::vector<V> goals()const{return route_points.empty()?std::vector<V>{{forward,left,height}}:route_points;}
  void validate()const{
    const double vals[]={height,forward,left,speed,acceleration,settle};
    for(double v:vals)if(!std::isfinite(v))throw std::runtime_error("nonfinite mission config");
    if(!std::isfinite(rescue_hover)||!std::isfinite(rescue_wait)||!std::isfinite(drop_timeout)||
       rescue_hover<2||rescue_hover>15||rescue_wait<rescue_hover+2||rescue_wait>60||drop_timeout<2||drop_timeout>15)
      throw std::runtime_error("invalid rescue hover/wait/drop limits");
    if(height<.6||height>1.4||speed<=0||speed>.2||acceleration<=0||acceleration>.2||settle<2||
       !flight.contains({forward,left,height})||!flight.contains({0,0,height}))throw std::runtime_error("goal/limits outside first-stage envelope");
    if(route_points.size()>32)throw std::runtime_error("route exceeds 32 goals");
    V previous{0,0,height};
    for(V goal:route_points){
      if(!flight.contains(goal)||std::abs(goal.z-height)>1e-6||norm(goal-previous)<.15)
        throw std::runtime_error("route needs distinct in-bounds goals at the configured cruise height");
      previous=goal;
    }
  }
};
class Mission {
 public:
  Settings settings;std::string state{"WAIT_INPUTS"},reason{"waiting for fresh navigation, map and recording"};
  V home{},goal{},target{},brake_velocity{},brake_start{};double yaw{},phase{},settled{-1},land_begin{},last_land{-10};
  bool triggered{},safe_to_stop{},observed{},previous_armed{};int attempts{};
  size_t route_index{};
  unsigned rescue_id{};bool rescue_attempted{},rescue_complete{};
  explicit Mission(Settings s={}):settings(s){settings.validate();}
  bool terminal()const{return state=="BLOCKED"||state=="TAKEN_OVER"||state=="ABORTED"||state=="COMPLETE";}
  bool active()const{return triggered&&!terminal();}
  V relative(V p)const{return rz(p-home,-yaw);}
  bool in_bounds(V p)const{
    // NED -> takeoff-relative forward/LEFT/UP.
    V d=rz(p-home,-yaw);return settings.flight.contains({d.x,-d.y,-d.z});
  }
  void block(const std::string& why){state="BLOCKED";reason=why;}
  void land_request(double now,const std::string& why){state="LAND_REQUEST";reason=why;land_begin=now;}
  Output step(const Input& i,const Spline* curve,const Alignment& alignment,const VoxelMap* map,
              bool route_safe,bool brake_safe){
    (void)map;
    Output out;out.position=target;out.yaw=yaw;
    safe_to_stop=i.seen&&i.fresh&&i.land_fresh&&i.landed&&!i.armed;
    const bool first=!observed&&i.seen,edge=observed&&!previous_armed&&i.armed;
    if(i.seen){observed=true;previous_armed=i.armed;}
    if(terminal())return out;
    if(first&&i.armed){block("already armed on first telemetry");return out;}
    if(active()&&i.fresh&&i.mode!=14){
      if((state=="LAND_REQUEST"||state=="LANDING")&&i.mode==18){
        state="LANDING";if(safe_to_stop){state="COMPLETE";reason="landed and disarmed confirmed";}
      }else{state="TAKEN_OVER";reason="mode left Offboard; control relinquished";}
      return out;
    }
    if(active()&&i.fresh&&!i.armed){state=safe_to_stop?"COMPLETE":"ABORTED";reason="disarmed; no further control";return out;}
    const bool nav=i.navigation&&i.fresh&&i.land_fresh&&!i.clock_fault&&!i.reset&&!i.backwards;
    if(state=="WAIT_INPUTS"){
      if(i.armed){block("arming before READY_TO_ARM");return out;}
      if(i.now>30){block("preparation timed out: "+i.problem);return out;}
      if(!safe_to_stop||!nav||!i.planning||!i.recording||!alignment.valid)return out;
      if(i.mode!=2){block("prepare requires Position, landed and disarmed");return out;}
      phase=i.now;state="PRESTREAM";reason="prestream ground hold; do not arm";
    }
    if(state=="PRESTREAM"||state=="READY_TO_ARM"){
      if(!nav||!i.planning||!i.recording){block("waiting gate lost: "+i.problem);return out;}
      if(i.armed){
        if(!(state=="READY_TO_ARM"&&edge&&i.mode==2&&i.landed)){block("invalid arming sequence");return out;}
        home=target=i.position;yaw=i.yaw;route_index=0;set_goal();
        phase=i.now;state="WAIT_OFFBOARD_ON_GROUND";reason="select Offboard on ground within 10 seconds";
      }else{
        if(!safe_to_stop||i.mode!=2){block("left Position/ground before arming");return out;}
        target=i.position;yaw=i.yaw;
        if(i.now-phase>=2){state="READY_TO_ARM";reason="ground gates passed; NOT flight clearance";}
        out.stream=true;out.position=target;out.yaw=yaw;return out;
      }
    }
    if(state=="WAIT_OFFBOARD_ON_GROUND"){
      if(!nav||!i.planning||!i.recording||!i.armed||!i.landed||norm(i.position-home)>.1||
         std::abs(angle(i.yaw-yaw))>5*pi/180||i.now-phase>=10){block("armed ground gate/10s deadline lost");return out;}
      if(i.mode!=2&&i.mode!=14){state="TAKEN_OVER";reason="other ground mode selected";return out;}
      if(i.mode!=14||!i.signal){out.stream=true;out.position=home;out.yaw=yaw;return out;}
      triggered=true;phase=i.now;state="CLIMB";reason="Offboard confirmed; climbing";
    }
    if(active()&&state!="LANDING"&&state!="LAND_REQUEST"){
      if(i.land_requested||!nav){land_request(i.now,i.land_requested?"operator Land request":"navigation fault: "+i.problem);}
    }
    if(settings.rescue_enabled&&!rescue_attempted&&i.person_confirmed&&
       (state=="FOLLOW_EGO"||state=="GOAL_SETTLE")&&nav&&i.planning&&i.recording&&i.signal){
      rescue_attempted=true;settled=-1;
      if(!brake_safe||!finite(i.velocity)||norm(i.velocity)>.4){
        begin_brake(i,brake_safe,"person confirmed without a verified stopping corridor");
      }else{
        state="RESCUE_BRAKE";reason="person confirmed; braking before stationary rescue hold";
        phase=i.now;brake_start=i.position;brake_velocity=i.velocity;
      }
    }
    if(state=="RESCUE_BRAKE"){
      if(!i.planning||!i.recording||!i.signal||!brake_safe){
        begin_brake(i,brake_safe,"rescue braking health lost");
      }else{
        const double speed=norm(brake_velocity),duration=speed/settings.acceleration;
        const double t=std::min(i.now-phase,duration);
        target=brake_start+brake_velocity*(speed>1e-8?t-.5*settings.acceleration*t*t/speed:0);
        out.stream=true;out.position=target;out.yaw=yaw;
        out.velocity=brake_velocity*(speed>1e-8?std::max(0.,1-settings.acceleration*t/speed):0);
        if(i.now-phase>=duration){state="RESCUE_HOLD";reason="waiting for continuous stationary rescue hold";phase=i.now;settled=-1;}
        return out;
      }
    }
    if(state=="RESCUE_HOLD"||state=="RESCUE_DROP"){
      const bool clear=alignment.valid&&map&&map->fresh(i.wall)&&map->free(alignment.to_map(target))&&in_bounds(target);
      const bool stationary=norm(i.position-target)<=.1&&norm(i.velocity)<.1&&std::abs(angle(i.yaw-yaw))<=5*pi/180;
      if(!i.planning||!i.recording||!i.signal||!clear){
        begin_brake(i,brake_safe,"rescue hover map/navigation/recording health lost");
      }else if(state=="RESCUE_HOLD"){
        settled=stationary?(settled<0?i.now:settled):-1;
        if(i.now-phase>=settings.rescue_wait){state="HOLD_FAULT";reason="rescue hold/executor readiness timeout; no drop retry";}
        else if(stationary&&settled>=0&&i.now-settled>=settings.rescue_hover&&i.actuator_ready){
          state="RESCUE_DROP";reason="stationary hold confirmed; one bounded drop request";
          phase=i.now;++rescue_id;out.drop=true;out.drop_id=rescue_id;
        }
        out.stream=true;out.position=target;out.yaw=yaw;return out;
      }else if(!stationary){
        begin_brake(i,brake_safe,"left stationary rescue hold during drop; cancel required");
      }else if(i.drop_id==rescue_id&&i.drop_result==DropResult::COMPLETE){
        rescue_complete=true;state="PLAN_GOAL";reason="drop completed and outputs off; requesting a fresh EGO trajectory";
        phase=i.now;settled=-1;out.stream=true;out.position=target;out.yaw=yaw;out.goal=true;return out;
      }else if((i.drop_id==rescue_id&&i.drop_result==DropResult::FAILED)||i.now-phase>=settings.drop_timeout){
        state="HOLD_FAULT";reason="drop failed/unconfirmed; hold for takeover, no retry or automatic resume";
        out.stream=true;out.position=target;out.yaw=yaw;return out;
      }else{out.stream=true;out.position=target;out.yaw=yaw;return out;}
    }
    if(state=="CLIMB"){
      if(!i.planning){land_request(i.now,"map/planner lost above cleared takeoff site");}
      else if(i.now-phase>30){land_request(i.now,"climb timeout");}
      else{
        target=home+V{0,0,-profile_position(i.now-phase,settings.height,settings.speed,settings.acceleration)};
        out.stream=true;out.position=target;out.yaw=yaw;
        out.velocity={0,0,-profile_velocity(i.now-phase,settings.height,settings.speed,settings.acceleration)};
        const bool close=norm(i.position-(home+V{0,0,-settings.height}))<=.1&&norm(i.velocity)<.1&&std::abs(angle(i.yaw-yaw))<=5*pi/180;
        settled=close?(settled<0?i.now:settled):-1;
        if(settled>=0&&i.now-settled>=settings.settle){state="PLAN_GOAL";reason="climb settled; requesting goal";phase=i.now;settled=-1;out.goal=true;}
        return out;
      }
    }
    if(state=="PLAN_GOAL"){
      if(!i.planning||i.now-phase>=5){
        if(route_index==0&&!rescue_complete)land_request(i.now,"no valid path within 5 seconds; Land at cleared takeoff site");
        else begin_brake(i,brake_safe,"next route goal unavailable; latch hold for takeover");
      }
      else if(curve&&curve->usable(i.wall)&&route_safe){state="FOLLOW_EGO";phase=i.now;reason="validated EGO trajectory";}
      else{out.stream=true;out.position=target;out.yaw=yaw;return out;}
    }
    if(state=="FOLLOW_EGO"||state=="GOAL_SETTLE"){
      const bool close=std::hypot(i.position.x-goal.x,i.position.y-goal.y)<=.15&&std::abs(i.position.z-goal.z)<=.1&&
        norm(i.velocity)<.1&&std::abs(angle(i.yaw-yaw))<=5*pi/180;
      const bool ended=curve&&i.wall>=curve->start+curve->duration();
      if(close&&(state=="GOAL_SETTLE"||ended)){
        if(state!="GOAL_SETTLE"){state="GOAL_SETTLE";settled=i.now;target=goal;reason="actual goal reached; settling";}
        if(!i.planning){begin_brake(i,brake_safe,"planning health lost at goal");}
        else if(i.now-settled>=settings.settle){
          if(route_index+1<settings.goals().size()){
            // Keep holding the reached goal while the NEW goal is planned.
            const V hold=goal;
            ++route_index;set_goal();phase=i.now;settled=-1;
            state="PLAN_GOAL";reason="route goal reached; requesting next EGO trajectory";
            out.stream=true;out.goal=true;out.position=target=hold;out.yaw=yaw;
            return out;
          }
          land_request(i.now,"final route goal settled; PX4 Land");
        }
        else{out.stream=true;out.position=goal;out.yaw=yaw;return out;}
      }else if(!i.planning||!curve||!curve->usable(i.wall)||!route_safe||i.now-phase>90||
               (curve&&norm(alignment.to_ned(curve->sample(i.wall))-i.position)>.35)){
        begin_brake(i,brake_safe,"planning/trajectory/map invalid; braking");
      }else{
        settled=-1;state="FOLLOW_EGO";target=alignment.to_ned(curve->sample(i.wall));
        out.stream=true;out.position=target;out.velocity=alignment.vector_to_ned(curve->speed(i.wall));
        out.acceleration=alignment.vector_to_ned(curve->acceleration.evaluate(curve->acceleration.begin()+i.wall-curve->start));
        out.yaw=yaw;return out;
      }
    }
    if(state=="BRAKE"){
      const double speed=norm(brake_velocity),duration=speed/settings.acceleration;
      const double t=std::min(i.now-phase,duration);
      target=brake_start+brake_velocity*(speed>1e-8?t-.5*settings.acceleration*t*t/speed:0);
      out.stream=true;out.position=target;out.yaw=yaw;
      out.velocity=brake_velocity*(speed>1e-8?std::max(0.,1-settings.acceleration*t/speed):0);
      if(i.now-phase>=duration){state="HOLD_FAULT";reason="fault latched; pilot takeover or Land required";}
      return out;
    }
    if(state=="HOLD_FAULT"){out.stream=true;out.position=target;out.yaw=yaw;return out;}
    if(state=="LAND_REQUEST"){
      if(!i.fresh||!i.land_fresh||i.clock_fault||i.now-land_begin>=5){state="ABORTED";reason="Land unconfirmed; stop heartbeat, rely on PX4 failsafe";return {};}
      out.stream=nav;out.position=target;out.yaw=yaw;
      if(attempts<3&&i.now-last_land>=1){out.land=true;++attempts;last_land=i.now;}return out;
    }
    return out;
  }
 private:
  void set_goal(){const V offset=settings.goals().at(route_index);goal=home+rz({offset.x,-offset.y,-offset.z},yaw);}
  void begin_brake(const Input& i,bool corridor,const std::string& why){
    // Do not invent a safe position hold when no verified stopping corridor exists.
    if(!corridor||!finite(i.velocity)||norm(i.velocity)>.4){land_request(i.now,"no verified stopping corridor; bounded Land request");return;}
    state="BRAKE";reason=why;phase=i.now;brake_start=i.position;brake_velocity=i.velocity;
  }
};
} // namespace kr260_ego
