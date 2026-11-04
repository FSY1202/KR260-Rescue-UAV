#pragma once
#include "voxel_map.hpp"
#include <chrono>
#include <functional>
#include <queue>

namespace kr260_ego {

// A necessary, deliberately permissive grid-connectivity check, NOT a path or
// spline validator. Diagonal neighbours are allowed; the unchanged complete
// curve/envelope checks must still reject corner cutting and collisions.
enum class Connectivity { Connected, Disconnected, EndpointBlocked, BudgetExceeded };
inline const char* connectivity_reason(Connectivity result) {
  switch(result) {
    case Connectivity::Connected:return "FREE cells connected (not a trajectory approval)";
    case Connectivity::Disconnected:return "FREE cells disconnected";
    case Connectivity::EndpointBlocked:return "start/target outside FREE cells";
    case Connectivity::BudgetExceeded:return "FREE connectivity budget exhausted (route unknown)";
  }
  return "invalid connectivity result";
}

class FreeConnectivity {
 public:
  using Clock=std::chrono::steady_clock;
  using Now=std::function<Clock::time_point()>;
  struct Limits { size_t max_expanded{20000};std::chrono::microseconds max_elapsed{5000}; };
  struct Result { Connectivity state{Connectivity::BudgetExceeded};size_t expanded{};bool cached{}; };

  Result query(const VoxelMap& map,V from,V to,Limits limits,Now now={}) {
    if(!now)now=[] {return Clock::now();};
    const auto begin=now();
    int sx,sy,sz,tx,ty,tz;
    if(!map.index(from,sx,sy,sz)||!map.index(to,tx,ty,tz))
      return {Connectivity::EndpointBlocked,0,false};
    const size_t start=map.address(sx,sy,sz),goal=map.address(tx,ty,tz);
    const Key key{&map,map.revision,start,goal,map.origin,map.resolution,map.nx,map.ny,map.nz,
                  limits.max_expanded,limits.max_elapsed.count()};
    if(cached_&&key==key_){auto value=result_;value.cached=true;return value;}
    const auto finish=[&](Connectivity state,size_t expanded) {
      key_=key;cached_=true;result_={state,expanded,false};return result_;
    };
    if(limits.max_expanded==0||limits.max_elapsed.count()<=0)
      return finish(Connectivity::BudgetExceeded,0);
    if(map.cells.size()!=size_t(map.nx)*map.ny*map.nz||
       map.cells[start]!=1||map.cells[goal]!=1)
      return finish(Connectivity::EndpointBlocked,0);
    if(start==goal)return finish(Connectivity::Connected,0);
    // Do not exhaust a flood-fill budget proving the common open 3 m route.
    // This remains only a necessary precondition, never a curve approval.
    if(map.line_free(from,to))
      return finish(now()-begin<limits.max_elapsed?Connectivity::Connected:Connectivity::BudgetExceeded,0);
    std::vector<uint8_t> seen(map.cells.size(),0);
    // Connectivity needs no shortest path. Goal-directed best-first traversal
    // avoids flooding a large open volume before trying a simple detour. It is
    // still complete if the bounded frontier can be exhausted; reaching the
    // budget yields UNKNOWN, not a false proof that no path exists.
    struct Node {size_t index;int distance;};
    struct Farther {bool operator()(const Node& a,const Node& b)const {
      return a.distance!=b.distance?a.distance>b.distance:a.index>b.index;
    }};
    const auto distance=[&](int x,int y,int z) {
      return (x-tx)*(x-tx)+(y-ty)*(y-ty)+(z-tz)*(z-tz);
    };
    std::priority_queue<Node,std::vector<Node>,Farther> queue;
    seen[start]=1;queue.push({start,distance(sx,sy,sz)});size_t expanded=0,queued=1;
    while(!queue.empty()) {
      if(expanded>=limits.max_expanded||now()-begin>=limits.max_elapsed)
        return finish(Connectivity::BudgetExceeded,expanded);
      const size_t index=queue.top().index;queue.pop();++expanded;
      const int z=int(index%map.nz),y=int((index/map.nz)%map.ny),x=int(index/(size_t(map.ny)*map.nz));
      for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz) {
        if((dx==0&&dy==0&&dz==0)||!map.inside(x+dx,y+dy,z+dz))continue;
        const size_t next=map.address(x+dx,y+dy,z+dz);
        if(seen[next]||map.cells[next]!=1)continue;
        if(next==goal)return finish(Connectivity::Connected,expanded);
        // Bound both explored work and the queued frontier/memory, independently
        // of the map's separate allocation cap. Exhaustion is NOT proof of no path.
        if(queued>=limits.max_expanded)return finish(Connectivity::BudgetExceeded,expanded);
        seen[next]=1;queue.push({next,distance(x+dx,y+dy,z+dz)});++queued;
      }
    }
    return finish(Connectivity::Disconnected,expanded);
  }
  void reset(){cached_=false;}

 private:
  struct Key {
    const VoxelMap* map;uint64_t revision;size_t start,goal;V origin;double resolution;
    int nx,ny,nz;size_t max_expanded;int64_t elapsed_us;
    bool operator==(const Key& other)const {
      return map==other.map&&revision==other.revision&&start==other.start&&goal==other.goal&&
        origin.x==other.origin.x&&origin.y==other.origin.y&&origin.z==other.origin.z&&
        resolution==other.resolution&&nx==other.nx&&ny==other.ny&&nz==other.nz&&
        max_expanded==other.max_expanded&&elapsed_us==other.elapsed_us;
    }
  };
  bool cached_{};Key key_{};Result result_{};
};

// Admission limits repeated work, not the duration of a single optimizer call.
// Map revision must advance before another expensive attempt can be admitted.
class RevisionAttemptGate {
 public:
  bool available(uint64_t revision)const {
    return revision&&(!claimed_||revision>revision_);
  }
  bool claim(uint64_t revision) {
    if(!available(revision))return false;
    claimed_=true;revision_=revision;++attempts_;return true;
  }
  uint64_t attempts()const{return attempts_;}
 private:
  bool claimed_{};uint64_t revision_{},attempts_{};
};
} // namespace kr260_ego
