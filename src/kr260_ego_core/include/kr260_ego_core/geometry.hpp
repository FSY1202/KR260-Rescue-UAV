#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace kr260_ego {
constexpr double pi=3.14159265358979323846;
inline double clamp(double x,double lo,double hi){return std::min(hi,std::max(lo,x));}
struct V { double x{},y{},z{}; };
inline V operator+(V a,V b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline V operator-(V a,V b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline V operator*(V a,double s){return {a.x*s,a.y*s,a.z*s};}
inline double norm(V a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
inline bool finite(V a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
inline V rz(V a,double yaw){return {std::cos(yaw)*a.x-std::sin(yaw)*a.y,std::sin(yaw)*a.x+std::cos(yaw)*a.y,a.z};}
// FAST-LIO gravity-aligned, arbitrary-heading map; NOT an assumed geographical ENU.
struct Alignment {
  bool valid{}, frozen{}; double delta{}; V translation{};
  V vector_to_ned(V p)const{return rz({p.x,-p.y,-p.z},delta);}
  V vector_to_map(V p)const{p=rz(p,-delta);return {p.x,-p.y,-p.z};}
  V to_ned(V p)const{return vector_to_ned(p)+translation;}
  V to_map(V p)const{return vector_to_map(p-translation);}
  double yaw_to_ned(double yaw)const{return angle(delta-yaw);}
  bool observe(V map,V ned,double map_yaw,double ned_yaw,double separation_s){
    if(frozen||!finite(map)||!finite(ned)||!std::isfinite(map_yaw)||!std::isfinite(ned_yaw)||
       !std::isfinite(separation_s)||std::abs(separation_s)>.05)return false;
    delta=angle(ned_yaw+map_yaw);translation=ned-vector_to_ned(map);valid=true;return true;
  }
  void freeze(){if(!valid)throw std::runtime_error("alignment missing");frozen=true;}
};
struct Bounds {
  V lower{-1,-2,.6},upper{4,2,1.4};
  bool contains(V p)const{return finite(p)&&p.x>=lower.x&&p.y>=lower.y&&p.z>=lower.z&&
    p.x<=upper.x&&p.y<=upper.y&&p.z<=upper.z;}
};
struct Curve {
  int degree{};std::vector<V> points;std::vector<double> knots;
  void validate()const{
    if(degree<0||degree>3||points.size()<static_cast<size_t>(degree+1)||points.size()>256||
       knots.size()!=points.size()+degree+1)throw std::runtime_error("invalid spline dimensions/order");
    for(V p:points)if(!finite(p))throw std::runtime_error("nonfinite spline control point");
    for(size_t i=0;i<knots.size();++i)if(!std::isfinite(knots[i])||(i&&knots[i]<knots[i-1]))
      throw std::runtime_error("invalid spline knots");
    if(end()-begin()<=1e-6||end()-begin()>60)throw std::runtime_error("invalid spline duration");
    for(size_t i=degree;i<points.size();++i)if(knots[i+1]-knots[i]<=1e-9)
      throw std::runtime_error("zero internal knot span");
  }
  double begin()const{return knots.at(degree);}
  double end()const{return knots.at(points.size());}
  V evaluate(double t)const{
    t=clamp(t,begin(),end());
    size_t k=points.size()-1;
    if(t<end())k=static_cast<size_t>(std::upper_bound(knots.begin()+degree,knots.begin()+points.size()+1,t)-knots.begin()-1);
    std::array<V,4> d{};
    for(int j=0;j<=degree;++j)d[j]=points[k-degree+j];
    for(int r=1;r<=degree;++r)for(int j=degree;j>=r;--j){
      const size_t i=k-degree+j;const double den=knots[i+degree-r+1]-knots[i];
      const double a=den>1e-12?(t-knots[i])/den:0;d[j]=d[j-1]*(1-a)+d[j]*a;
    }
    return d[degree];
  }
  Curve derivative()const{
    if(degree==0)throw std::runtime_error("constant derivative unsupported");
    Curve c;c.degree=degree-1;c.knots={knots.begin()+1,knots.end()-1};
    for(size_t i=0;i+1<points.size();++i){
      const double den=knots[i+degree+1]-knots[i+1];
      if(den<=1e-9)throw std::runtime_error("singular derivative");
      c.points.push_back((points[i+1]-points[i])*(degree/den));
    }
    c.validate();return c;
  }
};
struct Spline {
  Curve position,velocity,acceleration;double start{};long long id{-1};bool valid{};
  static Spline make(int order,std::vector<V> p,std::vector<double> u,double start_time,long long trajectory_id,
                     double max_speed=.2,double max_acceleration=.2){
    if(order!=3||!std::isfinite(start_time)||trajectory_id<0)throw std::runtime_error("only finite cubic trajectories accepted");
    Spline s;s.position={order,std::move(p),std::move(u)};s.position.validate();
    s.velocity=s.position.derivative();s.acceleration=s.velocity.derivative();
    // Convex-hull bounds cover the whole curve, not only a coarse sample grid.
    for(V v:s.velocity.points)if(norm(v)>max_speed+1e-4)throw std::runtime_error("spline exceeds speed limit");
    for(V a:s.acceleration.points)if(norm(a)>max_acceleration+1e-4)throw std::runtime_error("spline exceeds acceleration limit");
    s.start=start_time;s.id=trajectory_id;s.valid=true;return s;
  }
  double duration()const{return position.end()-position.begin();}
  bool usable(double now)const{return valid&&now>=start-.05&&now<=start+duration();}
  V sample(double now)const{return position.evaluate(position.begin()+now-start);}
  V speed(double now)const{return velocity.evaluate(velocity.begin()+now-start);}
};
inline double profile_position(double t,double length,double speed=.2,double acc=.2){
  const double ta=std::min(speed/acc,std::sqrt(length/acc)),v=acc*ta;
  const double tc=std::max(0.,length/v-ta),total=2*ta+tc;t=clamp(t,0.,total);
  if(t<ta)return .5*acc*t*t;
  if(t<ta+tc)return .5*acc*ta*ta+v*(t-ta);
  return length-.5*acc*(total-t)*(total-t);
}
inline double profile_velocity(double t,double length,double speed=.2,double acc=.2){
  const double ta=std::min(speed/acc,std::sqrt(length/acc)),v=acc*ta;
  const double tc=std::max(0.,length/v-ta),total=2*ta+tc;
  if(t<=0||t>=total)return 0;
  if(t<ta)return acc*t;
  if(t<ta+tc)return v;
  return acc*(total-t);
}
} // namespace kr260_ego
