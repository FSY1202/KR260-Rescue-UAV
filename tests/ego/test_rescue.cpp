#include <kr260_ego_core/mission.hpp>
#include <iostream>
#include <cstdlib>
using namespace kr260_ego;
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(0)
struct Flight {
  Mission m;Input i;Alignment a;VoxelMap map;
  Flight(){
    m.settings.rescue_enabled=true;m.triggered=true;m.observed=true;m.previous_armed=true;
    m.state="FOLLOW_EGO";m.goal={3,0,-1};m.target={0,0,-1};
    i.now=10;i.wall=100;i.seen=i.fresh=i.land_fresh=i.armed=i.navigation=i.planning=i.recording=i.signal=true;
    i.mode=14;i.position={0,0,-1};i.person_confirmed=true;
    CHECK(a.observe({0,0,1},{0,0,-1},0,0,0));a.freeze();
    std::fill(map.cells.begin(),map.cells.end(),1);map.source_time=i.wall;map.revision=1;
  }
  Output step(double dt=.05){i.now+=dt;i.wall+=dt;map.source_time=i.wall;return m.step(i,nullptr,a,&map,false,true);}
  void dropping(){
    CHECK(step().stream&&m.state=="RESCUE_HOLD");CHECK(step().stream);
    CHECK(!step(4.9).drop);i.actuator_ready=true;
    auto out=step(.11);CHECK(out.drop&&out.drop_id==1&&m.state=="RESCUE_DROP");
    CHECK(!step().drop); // Never retransmit a physical action as a new request.
  }
};
int main(){
  PersonObservation o{std::string(32,'a'),std::string(32,'b'),1,0,1000000000,true,.9};
  PersonConfirmation g;CHECK(!g.observe(o,o.source_ns));CHECK(g.count==1);
  CHECK(!g.observe(o,o.source_ns));CHECK(g.count==1);
  ++o.result;++o.frame;o.source_ns+=100000000;CHECK(g.observe(o,o.source_ns));
  CHECK(!g.observe(o,o.source_ns));CHECK(g.count==2);
  o.person=false;CHECK(!g.observe(o,o.source_ns));CHECK(g.count==0);
  o.person=true;++o.result;++o.frame;o.source_ns+=100000000;CHECK(!g.observe(o,o.source_ns));
  ++o.result;++o.frame;o.source_ns+=2100000000;CHECK(!g.observe(o,o.source_ns));CHECK(g.count==1);
  CHECK(!g.observe(o,o.source_ns+2000000001ULL));CHECK(g.count==0);
  CHECK(!g.observe(o,o.source_ns-1));
  {Flight f;f.dropping();f.i.drop_id=2;f.i.drop_result=DropResult::COMPLETE;CHECK(!f.step().goal&&f.m.state=="RESCUE_DROP");
   f.i.drop_id=1;auto out=f.step();CHECK(out.goal&&f.m.rescue_complete&&f.m.state=="PLAN_GOAL");
   CHECK(f.m.route_index==0&&norm(f.m.goal-V{3,0,-1})<1e-9);
   out=f.step(5.1);CHECK(!out.land&&f.m.state=="HOLD_FAULT");}
  {Flight f;f.dropping();auto out=f.step(5.1);CHECK(out.stream&&!out.goal&&f.m.state=="HOLD_FAULT");}
  {Flight f;f.dropping();f.i.drop_id=1;f.i.drop_result=DropResult::FAILED;CHECK(!f.step().goal&&f.m.state=="HOLD_FAULT");}
  {Flight f;f.dropping();f.i.mode=2;auto out=f.step();CHECK(!out.stream&&!out.drop&&f.m.state=="TAKEN_OVER");}
  {Flight f;f.dropping();f.i.navigation=false;CHECK(f.step().land&&f.m.state=="LAND_REQUEST");}
  {Flight f;f.step();f.step();CHECK(!f.step(5.1).drop);CHECK(f.m.state=="RESCUE_HOLD");
   CHECK(!f.step(15).drop&&f.m.state=="HOLD_FAULT");}
  {Flight f;f.i.person_confirmed=false;CHECK(!f.step().drop&&!f.m.rescue_attempted);}
  {Flight f;f.m.settings.rescue_enabled=false;CHECK(!f.step().drop&&!f.m.rescue_attempted);}
  std::cout<<"PASS "<<checks<<" rescue checks\n";
}
