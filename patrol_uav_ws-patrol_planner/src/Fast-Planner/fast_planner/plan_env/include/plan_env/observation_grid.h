#ifndef PLAN_ENV_OBSERVATION_GRID_H
#define PLAN_ENV_OBSERVATION_GRID_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fast_planner {

// No ROS, PCL, inference, clocks or threads here. All evidence uses source time.
class ObservationGrid {
public:
  using Point = std::array<double, 3>;
  enum State : uint8_t { UNKNOWN = 0, FREE = 1, RECENT = 2, STABLE = 3 };
  struct Config {
    Point origin{{0.,0.,0.}}, size{{0.,0.,0.}};
    double resolution = .10, range = 6., hit_lease = 1., free_lease = 2.;
    double stable_seconds = 2., stable_gap = .30, endpoint_guard = .20;
    unsigned clear_observations = 3, max_rays = 1500, max_cells = 600000;
  };
  struct Snapshot {
    Point origin{{0.,0.,0.}};
    std::array<int, 3> dims{{0,0,0}};
    double resolution = .1, stamp = 0., epoch = 0.;
    double work_ms=0.;
    size_t input_points=0;
    std::vector<uint8_t> states;
    // Summed-volume table of UNKNOWN cells: bounded body-coverage queries O(1).
    std::vector<uint32_t> unknown_prefix;
    std::vector<std::array<float,3>> obstacles;
    std::vector<uint8_t> column_eligible;
    int index(const Point& p) const {
      int a[3];
      for (int k=0;k<3;++k) {
        if (!std::isfinite(p[k])) return -1;
        const double value=std::floor((p[k]-origin[k])/resolution);
        if(value<0. || value>=dims[k]) return -1;
        a[k]=int(value);
      }
      return (a[0]*dims[1]+a[1])*dims[2]+a[2];
    }
    bool column(const Point& p) const {
      const int i=index(p);
      return i>=0 && states[size_t(i)]==STABLE;
    }
    bool covered(const Point& low, const Point& high) const {
      if (unknown_prefix.empty()) return false;
      int a[3],b[3];
      for (int k=0;k<3;++k) {
        if (!std::isfinite(low[k]) || !std::isfinite(high[k]) || low[k]>high[k]) return false;
        const double l=std::floor((low[k]-origin[k])/resolution);
        const double h=std::floor((high[k]-origin[k])/resolution)+1.;
        if(l<0. || h>dims[k]) return false;
        a[k]=int(l); b[k]=int(h);
      }
      auto at=[&](int x,int y,int z)->int64_t {
        return unknown_prefix[(size_t(x)*(dims[1]+1)+y)*(dims[2]+1)+z];
      };
      return at(b[0],b[1],b[2])-at(a[0],b[1],b[2])-at(b[0],a[1],b[2])-
          at(b[0],b[1],a[2])+at(a[0],a[1],b[2])+at(a[0],b[1],a[2])+
          at(b[0],a[1],a[2])-at(a[0],a[1],a[2])==0;
    }
  };

  explicit ObservationGrid(const Config& c) : config_(c) {
    for (int k=0;k<3;++k) {
      if (!std::isfinite(c.origin[k]) || !std::isfinite(c.size[k]) || c.size[k]<=0.)
        throw std::invalid_argument("invalid observation bounds");
    }
    const double values[]={c.resolution,c.range,c.hit_lease,c.free_lease,
                           c.stable_seconds,c.stable_gap,c.endpoint_guard};
    for(double v:values) if(!std::isfinite(v) || v<=0.)
      throw std::invalid_argument("invalid observation parameter");
    if(c.resolution<.05 || c.resolution>.25 || c.range>10. ||
       c.clear_observations<2 || c.clear_observations>255 ||
       !c.max_rays || c.max_rays>4000 || !c.max_cells || c.max_cells>600000 ||
       c.stable_gap>c.hit_lease || c.endpoint_guard<c.resolution)
      throw std::invalid_argument("observation work/memory bounds exceeded");
    size_t n=1;
    for(int k=0;k<3;++k) {
      const double d=std::ceil(c.size[k]/c.resolution);
      if(d>c.max_cells) throw std::invalid_argument("observation dimension too large");
      dims_[k]=int(d); n*=size_t(dims_[k]);
      if(n>c.max_cells) throw std::invalid_argument("observation grid too large");
    }
    cells_.resize(n); protected_frame_.assign(n,0);
  }
  void reset(double epoch) {
    if(!std::isfinite(epoch) || epoch<=0.) throw std::invalid_argument("invalid epoch");
    epoch_=epoch; stamp_=0.; frame_=0; ray_offset_=0;
    std::fill(cells_.begin(),cells_.end(),Cell());
    std::fill(protected_frame_.begin(),protected_frame_.end(),0);
    candidates_.clear();
  }
  // FreeDOM is a candidate mask, NEVER a source of hit/visibility timestamps.
  void candidates(const std::vector<Point>& points) {
    beginCandidates();
    for(const Point& p:points) candidate(p);
  }
  void beginCandidates() {
    for(size_t i:candidates_) cells_[i].candidate=false;
    candidates_.clear();
  }
  void candidate(const Point& p) {
      const int i=index(p);
      if(i>=0 && !cells_[size_t(i)].candidate) {
        cells_[size_t(i)].candidate=true; candidates_.push_back(size_t(i));
      }
  }
  bool observe(const std::vector<Point>& points, const Point& sensor,
               double stamp, bool do_clear) {
    if(!std::isfinite(stamp) || stamp<=epoch_ || stamp<=stamp_ || index(sensor)<0) return false;
    if(frame_==std::numeric_limits<uint32_t>::max()) return false; // require a new ground epoch
    ++frame_;
    const float t=float(stamp-epoch_);
    std::vector<Point> endpoints;
    endpoints.reserve(points.size());
    // Record ALL valid endpoints before ANY clearing. Duplicate points/rays
    // cannot increase temporal evidence within a single source frame.
    for(const Point& p:points) {
      double distance2=0.; bool finite=true;
      for(int k=0;k<3;++k) { finite=finite && std::isfinite(p[k]); distance2+=(p[k]-sensor[k])*(p[k]-sensor[k]); }
      if(!finite || !std::isfinite(distance2) || distance2<config_.resolution*config_.resolution) continue;
      endpoints.push_back(p); // out-of-grid returns still prove visibility up to the map boundary
      // A real farther return can prove free space within our local budget;
      // it must not create a synthetic occupied endpoint at the range limit.
      if(distance2>config_.range*config_.range) continue;
      const int i=index(p);
      if(i<0) continue;
      Cell& c=cells_[size_t(i)];
      if(c.hit_frame==frame_) continue;
      if(c.first<0.f || c.hit<0.f || c.misses>0 || t-c.hit>config_.stable_gap) c.first=t;
      c.hit=t; c.hit_frame=frame_; c.misses=0; c.cleared=false;
      const auto xyz=coordinates(size_t(i));
      for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) for(int z=-1;z<=1;++z) {
        const int j=address(xyz[0]+x,xyz[1]+y,xyz[2]+z);
        if(j>=0) protected_frame_[size_t(j)]=frame_;
      }
    }
    if(endpoints.empty()) return false;
    stamp_=stamp;
    if(!do_clear) return true;
    const size_t count=std::min(size_t(config_.max_rays),endpoints.size());
    // Stratified endpoint selection with a rotating offset; endpoint occupancy
    // above is never subsampled. Clearing stops BEFORE an actual return.
    for(size_t r=0;r<count;++r) {
      const Point& end=endpoints[(r*endpoints.size()/count+ray_offset_)%endpoints.size()];
      double length=0.; for(int k=0;k<3;++k) length+=(end[k]-sensor[k])*(end[k]-sensor[k]);
      length=std::sqrt(length);
      const double clear_length=std::min(length-config_.endpoint_guard,config_.range);
      for(double distance=0.;distance<clear_length;distance+=config_.resolution*.5) {
        Point p; for(int k=0;k<3;++k) p[k]=sensor[k]+(end[k]-sensor[k])*distance/length;
        const int i=index(p);
        if(i<0) break;
        if(protected_frame_[size_t(i)]==frame_) break;
        Cell& c=cells_[size_t(i)];
        if(c.miss_frame==frame_) continue;
        c.miss_frame=frame_;
        // Gaps do not accumulate indefinitely through occlusion.
        if(c.free<0.f || t-c.free>config_.free_lease) c.misses=0;
        c.free=t;
        if(c.misses<255) ++c.misses;
        if(c.misses>=config_.clear_observations) { c.cleared=true; c.first=-1.f; }
      }
    }
    ray_offset_=(ray_offset_+1)%endpoints.size();
    return true;
  }
  Snapshot snapshot() const {
    Snapshot out; out.origin=config_.origin; out.dims=dims_;
    out.resolution=config_.resolution; out.stamp=stamp_; out.epoch=epoch_;
    out.states.assign(cells_.size(),UNKNOWN);
    const double t=stamp_-epoch_;
    for(size_t i=0;i<cells_.size();++i) {
      const Cell& c=cells_[i];
      State state=UNKNOWN;
      if(c.cleared && c.free>=0.f && t-c.free<=config_.free_lease) state=FREE;
      else if(!c.cleared && c.hit>=0.f && t-c.hit<=config_.hit_lease) {
        state=c.candidate && c.first>=0.f && c.hit-c.first>=config_.stable_seconds ? STABLE : RECENT;
      }
      out.states[i]=state;
      if(state==RECENT || state==STABLE) {
        auto a=coordinates(i); std::array<float,3> p;
        for(int k=0;k<3;++k) p[k]=config_.origin[k]+(a[k]+.5)*config_.resolution;
        out.obstacles.push_back(p); out.column_eligible.push_back(state==STABLE);
      }
    }
    out.unknown_prefix.assign(size_t(dims_[0]+1)*(dims_[1]+1)*(dims_[2]+1),0);
    auto at=[&](int x,int y,int z)->uint32_t& {
      return out.unknown_prefix[(size_t(x)*(dims_[1]+1)+y)*(dims_[2]+1)+z];
    };
    for(int x=1;x<=dims_[0];++x) for(int y=1;y<=dims_[1];++y) for(int z=1;z<=dims_[2];++z) {
      const size_t i=size_t(address(x-1,y-1,z-1));
      const int64_t n=(out.states[i]==UNKNOWN)+int64_t(at(x-1,y,z))+at(x,y-1,z)+at(x,y,z-1)
          -int64_t(at(x-1,y-1,z))-at(x-1,y,z-1)-at(x,y-1,z-1)+at(x-1,y-1,z-1);
      at(x,y,z)=uint32_t(n);
    }
    return out;
  }
private:
  struct Cell {
    float hit=-1.f, free=-1.f, first=-1.f;
    uint32_t hit_frame=0, miss_frame=0;
    uint8_t misses=0;
    bool candidate=false, cleared=false;
  };
  Config config_;
  std::array<int,3> dims_;
  std::vector<Cell> cells_;
  std::vector<uint32_t> protected_frame_;
  std::vector<uint32_t> candidates_;
  double epoch_=0.,stamp_=0.;
  uint32_t frame_=0;
  size_t ray_offset_=0;
  int address(int x,int y,int z) const {
    if(x<0 || y<0 || z<0 || x>=dims_[0] || y>=dims_[1] || z>=dims_[2]) return -1;
    return (x*dims_[1]+y)*dims_[2]+z;
  }
  int index(const Point& p) const {
    int a[3];
    for(int k=0;k<3;++k) {
      if(!std::isfinite(p[k])) return -1;
      const double value=std::floor((p[k]-config_.origin[k])/config_.resolution);
      if(value<0. || value>=dims_[k]) return -1;
      a[k]=int(value);
    }
    return address(a[0],a[1],a[2]);
  }
  std::array<int,3> coordinates(size_t i) const {
    return {{int(i/(size_t(dims_[1])*dims_[2])),int((i/dims_[2])%dims_[1]),int(i%dims_[2])}};
  }
};
} // namespace fast_planner
#endif
