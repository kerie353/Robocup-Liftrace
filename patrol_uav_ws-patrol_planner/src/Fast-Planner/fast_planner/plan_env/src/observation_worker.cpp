#include <plan_env/observation_worker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace fast_planner {
namespace {
bool xyzLayout(const sensor_msgs::PointCloud2& msg, size_t limit) {
  if(msg.height!=1 || msg.width>limit || msg.is_bigendian || msg.point_step<12 ||
     uint64_t(msg.row_step)<uint64_t(msg.width)*msg.point_step ||
     uint64_t(msg.row_step)*msg.height>msg.data.size()) return false;
  for(const std::string name:{"x","y","z"}) {
    bool found=false;
    for(const auto& f:msg.fields) if(f.name==name && f.count==1 &&
        f.datatype==sensor_msgs::PointField::FLOAT32 && uint64_t(f.offset)+4<=msg.point_step) found=true;
    if(!found) return false;
  }
  return true;
}
}
ObservationWorker::ObservationWorker(ros::NodeHandle& nh, ObservationGrid::Config config,
                                     const std::string& frame)
    : grid_(config), frame_(frame), tf_buffer_(ros::Duration(3.)), tf_listener_(tf_buffer_) {
  std::string topic;
  nh.param<std::string>("sdf_map/observation/raw_topic",topic,"/cloud_registered_body");
  nh.param<std::string>("sdf_map/observation/sensor_frame",sensor_frame_,"body");
  nh.param("sdf_map/observation/max_age",max_age_,.35);
  double update_hz,clear_hz;
  nh.param("sdf_map/observation/update_hz",update_hz,10.);
  nh.param("sdf_map/observation/clear_hz",clear_hz,5.);
  if(!std::isfinite(max_age_) || max_age_<=0. || max_age_>1. ||
     !std::isfinite(update_hz) || update_hz<1. || update_hz>20. ||
     !std::isfinite(clear_hz) || clear_hz<1. || clear_hz>update_hz ||
     frame_.empty() || sensor_frame_.empty()) throw std::invalid_argument("invalid observation worker config");
  update_period_=1./update_hz; clear_period_=1./clear_hz;
  epoch_=std::max(1e-9,ros::Time::now().toSec()); grid_.reset(epoch_);
  sub_=nh.subscribe(topic,1,&ObservationWorker::submit,this);
  thread_=std::thread(&ObservationWorker::run,this);
}
ObservationWorker::~ObservationWorker() {
  stopping_=true; wake_.notify_all();
  if(thread_.joinable()) thread_.join();
}
void ObservationWorker::submit(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  std::lock_guard<std::mutex> lock(mutex_); pending_=cloud; wake_.notify_one();
}
void ObservationWorker::candidates(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  std::lock_guard<std::mutex> lock(mutex_); candidates_=cloud;
}
void ObservationWorker::reset(double epoch) {
  // Same lock order as run(): grid -> mailbox. A completed old frame cannot
  // publish after reset, and queued frames must pass the new source-time fence.
  std::lock_guard<std::mutex> grid_lock(grid_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  epoch_=epoch; grid_.reset(epoch); pending_.reset(); candidates_.reset(); latest_.reset();
}
std::shared_ptr<const ObservationGrid::Snapshot> ObservationWorker::snapshot() {
  std::lock_guard<std::mutex> lock(mutex_); return latest_;
}
void ObservationWorker::run() {
  double previous=0., last_clear=0., candidate_stamp=0.;
  ros::WallTime last_work;
  while(!stopping_) {
    sensor_msgs::PointCloud2ConstPtr cloud,candidate;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock,std::chrono::milliseconds(20),[&]{return stopping_ || bool(pending_);});
      if(stopping_) return;
      // Wall-time throttling also bounds work when replay /clock runs fast.
      if(!pending_ || (!last_work.isZero() && (ros::WallTime::now()-last_work).toSec()<update_period_)) {
        lock.unlock(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue;
      }
      cloud=pending_; pending_.reset(); candidate=candidates_; candidates_.reset();
    }
    last_work=ros::WallTime::now();
    try {
      const double stamp=cloud->header.stamp.toSec(),now=ros::Time::now().toSec();
      if(!xyzLayout(*cloud,max_input_points_) || cloud->header.frame_id!=sensor_frame_ || !std::isfinite(stamp) || stamp<=0. ||
         now-stamp<0. || now-stamp>max_age_ || !cloud->width || !cloud->height ||
         uint64_t(cloud->width)*cloud->height>max_input_points_ || cloud->data.size()>4*1024*1024)
        continue; // oversize never truncate possible dangerous endpoints
      // Bounded wait on this worker only, covering TF/topic scheduling order.
      // Never use latest TF and never block the planner's callback queue.
      const auto tf=tf_buffer_.lookupTransform(frame_,sensor_frame_,cloud->header.stamp,ros::Duration(.01));
      const auto& q=tf.transform.rotation; const auto& p=tf.transform.translation;
      Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      Eigen::Vector3d origin(p.x,p.y,p.z);
      if(!origin.allFinite() || !rotation.coeffs().allFinite() || std::abs(rotation.norm()-1.)>.01) continue;
      rotation.normalize();
      pcl::PointCloud<pcl::PointXYZ> body;
      pcl::fromROSMsg(*cloud,body);
      std::vector<ObservationGrid::Point> points;
      points.reserve(body.size());
      for(const auto& point:body) {
        const Eigen::Vector3d v=rotation*Eigen::Vector3d(point.x,point.y,point.z)+origin;
        if(v.allFinite()) points.push_back({{v.x(),v.y(),v.z()}});
      }
      std::lock_guard<std::mutex> grid_lock(grid_mutex_);
      if(stamp<=epoch_) continue;
      if(previous<epoch_) {previous=0.; last_clear=0.; candidate_stamp=0.;}
      if(stamp<=previous) continue; // duplicates and time rewind cannot renew evidence
      if(candidate && xyzLayout(*candidate,600000) && candidate->header.frame_id==frame_ &&
         candidate->header.stamp.toSec()>epoch_ && candidate->header.stamp.toSec()<=now &&
         now-candidate->header.stamp.toSec()<=2. && candidate->data.size()<=16*1024*1024 &&
         uint64_t(candidate->width)*candidate->height<=600000) {
        grid_.beginCandidates();
        if(candidate->width) {
          sensor_msgs::PointCloud2ConstIterator<float> x(*candidate,"x"),y(*candidate,"y"),z(*candidate,"z");
          for(uint32_t i=0;i<candidate->width;++i,++x,++y,++z) grid_.candidate({{*x,*y,*z}});
        }
        candidate_stamp=candidate->header.stamp.toSec();
      }
      if(candidate_stamp>0. && now-candidate_stamp>2.) {grid_.beginCandidates(); candidate_stamp=0.;}
      const bool clear=stamp-last_clear>=clear_period_;
      if(!grid_.observe(points,{{origin.x(),origin.y(),origin.z()}},stamp,clear)) continue;
      previous=stamp; if(clear) last_clear=stamp;
      auto result=std::make_shared<ObservationGrid::Snapshot>(grid_.snapshot());
      result->work_ms=(ros::WallTime::now()-last_work).toSec()*1000.;
      result->input_points=body.size();
      std::lock_guard<std::mutex> lock(mutex_); latest_=result;
    } catch(const tf2::TransformException& e) {
      ROS_WARN_STREAM_THROTTLE(2.,"Observation TF unavailable: "<<e.what());
    } catch(const std::exception& e) {
      ROS_ERROR_STREAM_THROTTLE(2.,"Observation frame rejected: "<<e.what());
    }
  }
}
}
