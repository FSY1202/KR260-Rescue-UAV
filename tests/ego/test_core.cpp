#include "kr260_ego_core/mission.hpp"
#include <iostream>
#include <cstdlib>
using namespace kr260_ego;
static int checks;
#define CHECK(x) do{++checks;if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(0)
template<class F> void rejects(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}CHECK(caught);}
Spline straight(double start=100,long long id=1){
  return Spline::make(3,{{0,0,1},{.1,0,1},{.2,0,1},{.3,0,1}},{0,0,0,0,3,3,3,3},start,id);
}
Input ground(){Input i;i.now=1;i.wall=100;i.seen=i.fresh=i.land_fresh=i.landed=i.navigation=i.planning=i.recording=i.signal=true;i.mode=2;i.position={4,7,-3};return i;}
void ready(Mission& m,Input& i,Alignment& a){a.valid=true;CHECK(m.step(i,nullptr,a,nullptr,false,false).stream);i.now+=2.01;CHECK(m.step(i,nullptr,a,nullptr,false,false).stream);CHECK(m.state=="READY_TO_ARM");}
void arm(Mission& m,Input& i,Alignment& a){ready(m,i,a);i.armed=true;i.now+=.05;CHECK(m.step(i,nullptr,a,nullptr,false,false).stream);CHECK(m.state=="WAIT_OFFBOARD_ON_GROUND");i.mode=14;i.now+=.1;CHECK(m.step(i,nullptr,a,nullptr,false,false).stream);CHECK(m.state=="CLIMB");}
int main(){
  for(double h:{-3.,0.,1.4})for(double n:{-2.,0.,1.5}){
    Alignment a;CHECK(a.observe({2,3,-1},{7,-4,h},h,n,.03));
    V p{4,-3,1};CHECK(norm(a.to_map(a.to_ned(p))-p)<1e-9);
    CHECK(std::abs(angle(a.yaw_to_ned(h)-n))<1e-9);a.freeze();CHECK(!a.observe({},{},0,0,0));
  }
  Alignment invalid;CHECK(!invalid.observe({},{},0,0,.051));rejects([&]{invalid.freeze();});
  auto s=straight();CHECK(s.usable(100)&&!s.usable(103.01));
  CHECK(norm(s.sample(100)-V{0,0,1})<1e-9);CHECK(norm(s.sample(103)-V{.3,0,1})<1e-9);
  for(int k=0;k<300;++k){CHECK(std::abs(s.speed(100+k*.01).x-.1)<1e-9);CHECK(norm(s.acceleration.evaluate(k*.01))<1e-9);}
  rejects([&]{Spline::make(2,s.position.points,s.position.knots,100,1);});
  rejects([&]{auto p=s.position.points;p[1].x=NAN;Spline::make(3,p,s.position.knots,100,1);});
  rejects([&]{auto u=s.position.knots;u[4]=-1;Spline::make(3,s.position.points,u,100,1);});
  rejects([&]{Spline::make(3,s.position.points,s.position.knots,100,1,.05,.2);});
  VoxelMap map({-2,-2,-.5},.1,40,40,30);
  CHECK(!map.free({0,0,1}));CHECK(!map.free({9,0,1}));
  std::vector<Cylinder> zone{{{0,0,0},1.3,-.4,2.4}};
  map.integrate({0,0,1},{{1.7,0,1}},100,.15,.1,zone);
  CHECK(map.free({0,0,1}));CHECK(!map.free({1.7,0,1}));CHECK(map.line_free({0,0,1},{.3,0,1}));
  CHECK(map.fresh(100.29)&&!map.fresh(100.31));
  CHECK(safe_curve(s,map,[](V){return true;},100,103));
  rejects([&]{map.integrate({0,0,1},{{1.7,0,1}},100,.15,.1);});
  VoxelMap copy({-2,-2,-.5},.1,40,40,30);copy.load_snapshot(map.origin,.1,40,40,30,map.cells,100,1);
  CHECK(copy.free({0,0,1}));rejects([&]{copy.load_snapshot(map.origin,.1,40,40,30,map.cells,100,1);});
  rejects([&]{VoxelMap huge({},.1,10000,10000,10000);});
  VoxelMap expire({-2,-2,-.5},.1,40,40,30);
  expire.integrate({0,0,1},{{1,0,1}},100,0,0);int x,y,z;CHECK(expire.index({1,0,1},x,y,z));
  CHECK(expire.cells[expire.address(x,y,z)]==2);
  expire.integrate({0,0,1},{{0,1,1}},103,0,0);CHECK(expire.cells[expire.address(x,y,z)]==0);
  // Prefix-sum erosion must be exactly equivalent to explicit cylinder offsets.
  for(double radius:{0.,.15,.4}){
    VoxelMap raw({-1.5,-1.5,-.5},.1,30,30,25),fast=raw;
    const std::vector<V> hits{{.5,.2,1},{-.7,0,.8},{0,.9,1.5}};
    const std::vector<Cylinder> clear{{{},1.1,-.4,1.9}};
    raw.integrate({0,0,1},hits,100,0,0,clear);fast.integrate({0,0,1},hits,100,radius,.2,clear);
    const int h=int(std::ceil(radius/.1));
    for(int xx=0;xx<30;++xx)for(int yy=0;yy<30;++yy)for(int zz=0;zz<25;++zz){
      bool free=raw.cells[raw.address(xx,yy,zz)]==1;
      for(int a=-h;a<=h&&free;++a)for(int b=-h;b<=h&&free;++b)if(std::hypot(double(a),double(b))*.1<=radius+.05)
        for(int c=-2;c<=2&&free;++c)free=raw.inside(xx+a,yy+b,zz+c)&&raw.cells[raw.address(xx+a,yy+b,zz+c)]==1;
      CHECK((fast.cells[fast.address(xx,yy,zz)]==1)==free);
    }
  }
  // The actual flight envelope is evaluated relative to nonzero home and heading.
  for(double yaw:{-pi/2,0.,pi/2}){
    Mission m;auto i=ground();i.yaw=yaw;Alignment a;arm(m,i,a);
    CHECK(m.in_bounds(m.home+rz({3,0,-1},yaw)));CHECK(!m.in_bounds(m.home+rz({0,3,-1},yaw)));
    i.now+=7;i.position=m.home+V{0,0,-1};m.step(i,nullptr,a,nullptr,false,false);i.now+=2.1;m.step(i,nullptr,a,nullptr,false,false);
    CHECK(m.state=="PLAN_GOAL");
    a.observe({0,0,1},i.position,0,yaw,0);a.freeze();
    Spline path=Spline::make(3,{{0,0,1},{1,0,1},{2,0,1},{3,0,1}},{0,0,0,0,30,30,30,30},i.wall,5);
    auto out=m.step(i,&path,a,nullptr,true,true);CHECK(m.state=="FOLLOW_EGO"&&out.stream);
    i.planning=false;i.velocity=rz({.1,0,0},yaw);i.now+=.05;i.wall+=.05;
    out=m.step(i,&path,a,nullptr,false,true);CHECK(m.state=="BRAKE"&&out.stream&&!out.land);
    i.now+=1.;out=m.step(i,&path,a,nullptr,false,true);CHECK(m.state=="HOLD_FAULT"&&out.stream);
    i.planning=true;i.now+=1;CHECK(m.step(i,&path,a,nullptr,true,true).stream&&m.state=="HOLD_FAULT");
    i.land_requested=true;CHECK(m.step(i,&path,a,nullptr,true,true).land);i.mode=18;m.step(i,&path,a,nullptr,true,true);
    i.armed=false;i.landed=true;m.step(i,&path,a,nullptr,true,true);CHECK(m.state=="COMPLETE");
  }
  {Mission m;auto i=ground();Alignment a;ready(m,i,a);i.mode=14;i.now+=.05;CHECK(!m.step(i,nullptr,a,nullptr,false,false).stream);CHECK(m.state=="BLOCKED");}
  for(int failure=0;failure<7;++failure){
    Mission m;auto i=ground();Alignment a;ready(m,i,a);i.armed=true;i.now+=.05;m.step(i,nullptr,a,nullptr,false,false);
    if(failure==0)i.now+=10;
    if(failure==1)i.landed=false;
    if(failure==2)i.position.x+=.101;
    if(failure==3)i.yaw+=6*pi/180;
    if(failure==4)i.navigation=false;
    if(failure==5)i.mode=15;
    if(failure==6)i.armed=false;
    auto o=m.step(i,nullptr,a,nullptr,false,false);CHECK(!o.land&&!o.stream&&!m.triggered);CHECK(m.terminal());
  }
  {Mission m;auto i=ground();Alignment a;arm(m,i,a);i.now+=1;i.mode=2;auto o=m.step(i,nullptr,a,nullptr,false,false);CHECK(m.state=="TAKEN_OVER"&&!o.stream&&!o.land);i.mode=14;CHECK(!m.step(i,nullptr,a,nullptr,false,false).stream);}
  {Mission m;auto i=ground();i.yaw=pi/2;Alignment a;arm(m,i,a);CHECK(norm(m.goal-V{4,10,-4})<1e-9);
   i.now+=7;i.position={4,7,-4};m.step(i,nullptr,a,nullptr,false,false);i.now+=2.1;auto o=m.step(i,nullptr,a,nullptr,false,false);CHECK(m.state=="PLAN_GOAL"&&o.goal);
   i.now+=5.1;o=m.step(i,nullptr,a,nullptr,false,false);CHECK(m.state=="LAND_REQUEST"&&o.land);
   for(int k=0;k<4;++k){i.now+=1.01;m.step(i,nullptr,a,nullptr,false,false);}CHECK(m.attempts==3);i.now+=1;CHECK(!m.step(i,nullptr,a,nullptr,false,false).stream);CHECK(m.state=="ABORTED");}
  {Mission m;auto i=ground();Alignment a;arm(m,i,a);i.reset=true;i.now+=.1;auto o=m.step(i,nullptr,a,nullptr,false,false);CHECK(o.land&&!o.stream);i.mode=18;i.landed=false;m.step(i,nullptr,a,nullptr,false,false);CHECK(m.state=="LANDING");i.armed=false;i.landed=true;m.step(i,nullptr,a,nullptr,false,false);CHECK(m.state=="COMPLETE");}
  std::cout<<"PASS "<<checks<<" core checks\n";
}
