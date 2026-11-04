#pragma once
#include "geometry.hpp"
#include <cstdint>
#include <functional>
#include <limits>

namespace kr260_ego {
struct Cylinder { V center{}; double radius{}, bottom{}, top{}; };
// Fixed-size local map. Expired observations become UNKNOWN, never FREE.
class VoxelMap {
 public:
  V origin{-5,-5,-.5};double resolution{.1};int nx{100},ny{100},nz{30};
  double source_time{-1e9};uint64_t revision{};
  std::vector<uint8_t> cells;
  VoxelMap(){allocate();}
  VoxelMap(V o,double r,int x,int y,int z):origin(o),resolution(r),nx(x),ny(y),nz(z){allocate();}
  void allocate(){
    if(!finite(origin)||!std::isfinite(resolution)||resolution<.05||resolution>.2||
       nx<1||ny<1||nz<1||nx>200||ny>200||nz>100||size_t(nx)*ny*nz>1000000)
      throw std::runtime_error("unsafe map allocation");
    const size_t n=size_t(nx)*ny*nz;cells.assign(n,0);odds_.assign(n,0);seen_.assign(n,-1e9);
  }
  bool index(V p,int& x,int& y,int& z)const{
    if(!finite(p))return false;
    x=int(std::floor((p.x-origin.x)/resolution));y=int(std::floor((p.y-origin.y)/resolution));
    z=int(std::floor((p.z-origin.z)/resolution));return inside(x,y,z);
  }
  bool inside(int x,int y,int z)const{return x>=0&&y>=0&&z>=0&&x<nx&&y<ny&&z<nz;}
  size_t address(int x,int y,int z)const{return (size_t(x)*ny+y)*nz+z;}
  V center(int x,int y,int z)const{return origin+V{(x+.5)*resolution,(y+.5)*resolution,(z+.5)*resolution};}
  bool free(V p)const{int x,y,z;return index(p,x,y,z)&&cells[address(x,y,z)]==1;}
  bool fresh(double now,double limit=.3)const{return now-source_time>=-.01&&now-source_time<=limit;}
  bool line_free(V a,V b)const{
    if(!finite(a)||!finite(b)||norm(b-a)>20)return false;
    const int n=std::max(1,int(std::ceil(norm(b-a)/(resolution*.4))));
    for(int i=0;i<=n;++i)if(!free(a+(b-a)*(double(i)/n)))return false;
    return true;
  }
  void integrate(V sensor,const std::vector<V>& points,double stamp,double radius,double half_height,
                 const std::vector<Cylinder>& declared_clear={}){
    int sx,sy,sz;
    if(!index(sensor,sx,sy,sz)||points.empty()||points.size()>50000||!std::isfinite(stamp)||
       stamp<=source_time||radius<0||radius>.9||half_height<0||half_height>.8)
      throw std::runtime_error("invalid/duplicate map input");
    std::vector<size_t> hits;hits.reserve(points.size());
    for(V point:points){
      if(!finite(point)||norm(point-sensor)<.1)continue;
      const V diff=point-sensor;const double length=norm(diff);const double t_end=std::min(1.,6./length);
      V end=sensor+diff*t_end;
      // Clip to map extent, conservatively leaving outside endpoints unoccupied.
      double t=t_end;
      const double s[3]={sensor.x,sensor.y,sensor.z},d[3]={diff.x,diff.y,diff.z};
      const double lo[3]={origin.x,origin.y,origin.z};const int dim[3]={nx,ny,nz};
      for(int a=0;a<3;++a)if(std::abs(d[a])>1e-12){
        double bound=d[a]>0?lo[a]+dim[a]*resolution-1e-6:lo[a]+1e-6;
        t=std::min(t,(bound-s[a])/d[a]);
      }
      if(t<=0)continue;
      end=sensor+diff*t;
      int ex,ey,ez;if(!index(end,ex,ey,ez))continue;
      const bool endpoint=t>.999999;
      int x=sx,y=sy,z=sz;
      const int step[3]={diff.x>=0?1:-1,diff.y>=0?1:-1,diff.z>=0?1:-1};
      int xyz[3]={x,y,z};double next[3],delta[3];
      for(int a=0;a<3;++a){
        if(std::abs(d[a])<1e-12){next[a]=delta[a]=std::numeric_limits<double>::infinity();continue;}
        const double boundary=lo[a]+(xyz[a]+(step[a]>0?1:0))*resolution;
        next[a]=(boundary-s[a])/d[a];delta[a]=resolution/std::abs(d[a]);
      }
      for(int n=0;n<nx+ny+nz+3;++n){
        x=xyz[0];y=xyz[1];z=xyz[2];if(!inside(x,y,z))break;
        const auto adr=address(x,y,z);
        if(x==ex&&y==ey&&z==ez){if(endpoint)hits.push_back(adr);else miss(adr,stamp);break;}
        miss(adr,stamp);int a=next[0]<next[1]?0:1;if(next[2]<next[a])a=2;
        xyz[a]+=step[a];next[a]+=delta[a];
      }
    }
    std::sort(hits.begin(),hits.end());hits.erase(std::unique(hits.begin(),hits.end()),hits.end());
    for(auto i:hits){odds_[i]=static_cast<int8_t>(std::min(4,int(odds_[i])+3));seen_[i]=stamp;}
    std::vector<uint8_t> raw(cells.size());
    // The radial predicate is independent of z. Compute its original expression
    // lazily at the first eligible unknown voxel, once per XY/declared column.
    std::vector<int8_t> column_xy(declared_clear.size());
    for(int x=0;x<nx;++x)for(int y=0;y<ny;++y){
      std::fill(column_xy.begin(),column_xy.end(),int8_t(-1));
      for(int z=0;z<nz;++z){
        const auto i=address(x,y,z);const double age=stamp-seen_[i];
        if(age>2){odds_[i]=0;raw[i]=0;}else raw[i]=odds_[i]>0?2:(odds_[i]<0?1:0);
        // Operator-declared start/landing columns bootstrap only unknown cells.
        // A measured occupied voxel ALWAYS overrides a declaration.
        if(raw[i]==0)for(size_t ci=0;ci<declared_clear.size();++ci){
          const auto& c=declared_clear[ci];const V p=center(x,y,z);
          if(p.z>=c.bottom&&p.z<=c.top){
            auto& radial=column_xy[ci];
            if(radial<0)radial=std::hypot(p.x-c.center.x,p.y-c.center.y)<=c.radius;
            if(radial){raw[i]=1;break;}
          }
        }
      }
    }
    const int hr=int(std::ceil(radius/resolution)),vr=int(std::ceil(half_height/resolution));
    // Separable cylinder erosion. Prefix sums reduce O(volume*r^2*h) to
    // O(volume*r) without changing the previous explicit-offset result.
    std::vector<uint8_t> vertical(raw.size());
    // Entry zero remains zero; entries 1..nz are overwritten for every XY.
    std::vector<int> sums(nz+1,0);
    for(int x=0;x<nx;++x)for(int y=0;y<ny;++y){
      for(int z=0;z<nz;++z)sums[z+1]=sums[z]+(raw[address(x,y,z)]!=1);
      for(int z=vr;z<nz-vr;++z)vertical[address(x,y,z)]=sums[z+vr+1]==sums[z-vr];
    }
    std::vector<uint32_t> row_prefix(size_t(nx+1)*ny*nz,0);
    const auto row=[&](int x,int y,int z){return (size_t(x)*ny+y)*nz+z;};
    for(int x=0;x<nx;++x)for(int y=0;y<ny;++y)for(int z=0;z<nz;++z)
      row_prefix[row(x+1,y,z)]=row_prefix[row(x,y,z)]+vertical[address(x,y,z)];
    std::vector<std::array<int,2>> rows;
    for(int b=-hr;b<=hr;++b){
      int extent=-1;
      for(int a=0;a<=hr;++a)if(std::hypot(double(a),double(b))*resolution<=radius+resolution*.5)extent=a;
      if(extent>=0)rows.push_back({{b,extent}});
    }
    // Erode known free space by the swept airframe envelope: unknown is blocked too.
    cells=raw;
    for(int x=0;x<nx;++x)for(int y=0;y<ny;++y)for(int z=0;z<nz;++z){
      const auto i=address(x,y,z);if(raw[i]!=1)continue;
      for(const auto& r:rows){
        const int yy=y+r[0],extent=r[1];
        if(yy<0||yy>=ny||x-extent<0||x+extent>=nx||
           row_prefix[row(x+extent+1,yy,z)]-row_prefix[row(x-extent,yy,z)]!=uint32_t(2*extent+1)){
          cells[i]=0;break;
        }
      }
    }
    source_time=stamp;++revision;
  }
  void load_snapshot(V o,double r,int x,int y,int z,const std::vector<uint8_t>& data,double stamp,uint64_t rev){
    if(x!=nx||y!=ny||z!=nz||r!=resolution||norm(o-origin)>1e-5)throw std::runtime_error("map geometry changed");
    if(data.size()!=cells.size()||rev<=revision||!std::isfinite(stamp)||stamp<=source_time)
      throw std::runtime_error("invalid/repeated map snapshot");
    for(uint8_t c:data)if(c>2)throw std::runtime_error("unknown map encoding");
    cells=data;source_time=stamp;revision=rev;
  }
 private:
  std::vector<int8_t> odds_;std::vector<double> seen_;
  void miss(size_t i,double stamp){
    // At most one free-space vote per voxel per scan; ray density is not evidence.
    if(seen_[i]==stamp)return;
    odds_[i]=static_cast<int8_t>(std::max(-4,int(odds_[i])-1));seen_[i]=stamp;
  }
};
inline bool safe_curve(const Spline& s,const VoxelMap& map,const std::function<bool(V)>& bounds,double from,double until){
  if(!s.valid||until<from)return false;
  until=std::min(until,s.start+s.duration());V last=s.sample(from);
  for(double t=from;t<until+.025;t+=.025){const V p=s.sample(std::min(t,until));
    if(!bounds(p)||!map.line_free(last,p))return false;
    last=p;}
  return true;
}
} // namespace kr260_ego
