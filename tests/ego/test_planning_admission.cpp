#include "kr260_ego_core/planning_admission.hpp"
#include <cstdlib>
#include <iostream>
#include <queue>
using namespace kr260_ego;
static int checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';std::exit(1);}}while(0)
static FreeConnectivity::Limits generous(){return {10000,std::chrono::seconds(2)};}
static V point(const VoxelMap& m,int x,int y,int z){return m.center(x,y,z);}
static bool reference(const VoxelMap& m,V a,V b){
  int sx,sy,sz,tx,ty,tz;
  if(!m.index(a,sx,sy,sz)||!m.index(b,tx,ty,tz))return false;
  const auto goal=m.address(tx,ty,tz),start=m.address(sx,sy,sz);
  if(m.cells[start]!=1||m.cells[goal]!=1)return false;
  std::queue<std::array<int,3>> q;q.push({{sx,sy,sz}});
  std::vector<bool> seen(m.cells.size(),false);seen[start]=true;
  while(!q.empty()){
    auto p=q.front();q.pop();if(m.address(p[0],p[1],p[2])==goal)return true;
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){
      const int x=p[0]+dx,y=p[1]+dy,z=p[2]+dz;
      if(!m.inside(x,y,z))continue;
      const auto next=m.address(x,y,z);
      if(m.cells[next]==1&&!seen[next]){seen[next]=true;q.push({{x,y,z}});}
    }
  }
  return false;
}
int main(){
  VoxelMap m({},.1,9,7,5);m.revision=1;
  const V start=point(m,0,3,2),goal=point(m,8,3,2);
  FreeConnectivity probe;
  const auto original=m.cells;
  CHECK(probe.query(m,start,goal,generous()).state==Connectivity::EndpointBlocked);
  CHECK(m.cells==original);
  std::fill(m.cells.begin(),m.cells.end(),1);++m.revision;
  auto result=probe.query(m,start,goal,generous());
  CHECK(result.state==Connectivity::Connected&&!result.cached);
  CHECK(probe.query(m,start,goal,generous()).cached);
  for(uint8_t blocked:{uint8_t(0),uint8_t(2)}){
    for(int y=0;y<m.ny;++y)for(int z=0;z<m.nz;++z)m.cells[m.address(4,y,z)]=blocked;
    ++m.revision;const auto before=m.cells;
    CHECK(probe.query(m,start,goal,generous()).state==Connectivity::Disconnected);
    CHECK(m.cells==before);CHECK(probe.query(m,start,goal,generous()).cached);
  }
  // Opening one real FREE voxel creates a grid route, not a flight corridor.
  m.cells[m.address(4,0,2)]=1;++m.revision;
  CHECK(probe.query(m,start,goal,generous()).state==Connectivity::Connected);
  CHECK(!probe.query(m,point(m,0,0,0),goal,generous()).cached);
  CHECK(!probe.query(m,start,point(m,8,0,0),generous()).cached);
  CHECK(probe.query(m,{-1,0,0},goal,generous()).state==Connectivity::EndpointBlocked);
  CHECK(probe.query(m,start,{NAN,0,0},generous()).state==Connectivity::EndpointBlocked);
  CHECK(probe.query(m,start,start,generous()).state==Connectivity::Connected);
  CHECK(probe.query(m,start,goal,{1,std::chrono::seconds(2)}).state==Connectivity::BudgetExceeded);
  CHECK(probe.query(m,start,goal,{0,std::chrono::seconds(2)}).state==Connectivity::BudgetExceeded);
  CHECK(probe.query(m,start,goal,{1000,std::chrono::microseconds(0)}).state==Connectivity::BudgetExceeded);
  unsigned ticks=0;
  auto clock=[&]{return FreeConnectivity::Clock::time_point{}+std::chrono::milliseconds(ticks++);};
  ++m.revision;
  CHECK(probe.query(m,start,goal,{1000,std::chrono::microseconds(500)},clock).state==Connectivity::BudgetExceeded);
  auto cached_budget=probe.query(m,start,goal,{1000,std::chrono::microseconds(500)},clock);
  CHECK(cached_budget.cached&&cached_budget.state==Connectivity::BudgetExceeded);
  // Different budgets invalidate the cache, not a permanently negative answer.
  CHECK(probe.query(m,start,goal,generous()).state==Connectivity::Connected);
  VoxelMap diagonal({},.1,3,3,3);diagonal.revision=1;
  diagonal.cells[diagonal.address(0,0,0)]=diagonal.cells[diagonal.address(1,1,1)]=1;
  CHECK(probe.query(diagonal,point(diagonal,0,0,0),point(diagonal,1,1,1),generous()).state==Connectivity::Connected);
  // The real-size open 3 m scene must not hit a spherical BFS's 20k frontier.
  VoxelMap open;open.revision=1;
  std::fill(open.cells.begin(),open.cells.end(),1);
  auto open_path=probe.query(open,{0,0,1},{3,0,1},{});
  CHECK(open_path.state==Connectivity::Connected&&open_path.expanded==0);
  // A full-height obstacle requires an XY detour, with unchanged 20k/5ms limits.
  for(int x=62;x<=67;++x)for(int y=44;y<=55;++y)for(int z=0;z<open.nz;++z)
    open.cells[open.address(x,y,z)]=2;
  ++open.revision;const auto obstacle=open.cells;
  auto detour=probe.query(open,{0,0,1},{3,0,1},{});
  CHECK(detour.state==Connectivity::Connected&&detour.expanded>0&&detour.expanded<20000);
  CHECK(open.cells==obstacle);
  // Independent randomized 26-neighbour oracle: UNKNOWN and OCCUPIED both block.
  uint32_t random=1234567;
  for(int trial=0;trial<160;++trial){
    for(auto& c:m.cells){random=random*1664525u+1013904223u;c=uint8_t((random>>16)%3);}
    ++m.revision;
    const auto before=m.cells;
    const bool expected=reference(m,start,goal);
    const auto actual=probe.query(m,start,goal,generous());
    CHECK(actual.state!=Connectivity::BudgetExceeded);
    CHECK((actual.state==Connectivity::Connected)==expected);CHECK(m.cells==before);
  }
  // One expensive attempt per advancing map, including failed/unknown outcomes.
  RevisionAttemptGate gate;
  CHECK(!gate.available(0));CHECK(gate.available(1));
  CHECK(!gate.claim(0));CHECK(gate.attempts()==0);
  CHECK(gate.claim(1));CHECK(!gate.available(1));CHECK(gate.available(2));
  CHECK(!gate.claim(1));CHECK(!gate.claim(0));
  CHECK(gate.claim(2));CHECK(!gate.available(1));CHECK(!gate.available(2));
  CHECK(gate.available(3));CHECK(!gate.claim(1));CHECK(gate.attempts()==2);
  for(int i=0;i<100;++i)CHECK(!gate.claim(2));
  CHECK(gate.claim(99));CHECK(gate.attempts()==3);
  std::cout<<"PASS "<<checks<<" planning admission checks\n";
}
