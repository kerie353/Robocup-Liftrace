#include <plan_env/sdf_map.h>
#include <std_msgs/Empty.h>
#include <sstream>

void SDFMap::initObservation() {
  node_.param("/navigation_observation/enabled",observation_enabled_,false);
  if(!observation_enabled_) return;
  fast_planner::ObservationGrid::Config c;
  for(int k=0;k<3;++k) {c.origin[k]=mp_.map_origin_[k]; c.size[k]=mp_.map_size_[k];}
  c.resolution=mp_.resolution_;
  node_.param("sdf_map/observation/range",c.range,6.);
  node_.param("sdf_map/observation/hit_lease",c.hit_lease,1.);
  node_.param("sdf_map/observation/free_lease",c.free_lease,2.);
  node_.param("sdf_map/observation/stable_seconds",c.stable_seconds,2.);
  node_.param("sdf_map/observation/stable_gap",c.stable_gap,.30);
  node_.param("sdf_map/observation/endpoint_guard",c.endpoint_guard,.20);
  int observations,rays;
  node_.param("sdf_map/observation/clear_observations",observations,3);
  node_.param("sdf_map/observation/max_rays",rays,1500);
  if(observations<2 || observations>255 || rays<1 || rays>4000)
    throw std::invalid_argument("invalid observation count or ray budget");
  c.clear_observations=unsigned(observations); c.max_rays=unsigned(rays);
  node_.param("sdf_map/observation/warmup_seconds",observation_warmup_,2.);
  node_.param("sdf_map/observation/body_xy",observation_body_xy_,.275);
  node_.param("sdf_map/observation/body_up",observation_body_up_,.20);
  node_.param("sdf_map/observation/body_down",observation_body_down_,.20);
  const double values[]={observation_warmup_,observation_body_xy_,observation_body_up_,observation_body_down_};
  for(double v:values) if(!std::isfinite(v) || v<=0.) throw std::invalid_argument("invalid observation coverage envelope");
  observation_epoch_=ros::Time::now().toSec();
  node_.setParam("/navigation_observation/rebuilding",false);
  observation_worker_.reset(new fast_planner::ObservationWorker(node_,c,mp_.frame_id_));
  observation_cloud_pub_=node_.advertise<sensor_msgs::PointCloud2>("/navigation/observed_obstacles",1);
  observation_ready_pub_=node_.advertise<std_msgs::Header>("/navigation/observation_ready",1,true);
  observation_diagnostics_pub_=node_.advertise<std_msgs::String>("/navigation/observation_diagnostics",1);
  observation_fc_sub_=node_.subscribe<mavros_msgs::State>("/mavros/state",1,
      [this](const mavros_msgs::State::ConstPtr& msg){observation_fc_=*msg; observation_fc_received_=ros::Time::now();});
  observation_landed_sub_=node_.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",1,
      [this](const mavros_msgs::ExtendedState::ConstPtr& msg){observation_landed_=*msg; observation_landed_received_=ros::Time::now();});
  observation_reset_srv_=node_.advertiseService("/navigation/rebuild_map",&SDFMap::rebuildObservation,this);
  observation_timer_=node_.createTimer(ros::Duration(.10),&SDFMap::observationTimer,this);
  invalidateObservation();
}

bool SDFMap::observationFresh() const {
  if(!observation_enabled_) return true;
  if(observation_resetting_ || !observation_snapshot_) return false;
  const double pose_age=(ros::Time::now()-observation_odom_stamp_).toSec();
  if(!md_.has_odom_ || observation_odom_stamp_.isZero() || pose_age<0. ||
     pose_age>observation_worker_->maxAge()) return false;
  const double age=ros::Time::now().toSec()-observation_snapshot_->stamp;
  return observation_snapshot_->stamp>observation_epoch_ && age>=0. && age<=observation_worker_->maxAge();
}
bool SDFMap::observationCovered(const Eigen::Vector3d& p) const {
  if(!observation_enabled_) return true;
  if(!observationFresh() || !p.allFinite()) return false;
  const Eigen::Vector3d low=p-Eigen::Vector3d(observation_body_xy_,observation_body_xy_,observation_body_down_);
  const Eigen::Vector3d high=p+Eigen::Vector3d(observation_body_xy_,observation_body_xy_,observation_body_up_);
  if((low.array()<observation_min_.array()).any() || (high.array()>observation_max_.array()).any()) return false;
  return observation_snapshot_->covered(
      {{p.x()-observation_body_xy_,p.y()-observation_body_xy_,p.z()-observation_body_down_}},
      {{p.x()+observation_body_xy_,p.y()+observation_body_xy_,p.z()+observation_body_up_}});
}
bool SDFMap::observationColumn(const pcl::PointXYZ& p) const {
  return !observation_enabled_ || (observation_snapshot_ &&
      observation_snapshot_->column({{p.x,p.y,p.z}}));
}
void SDFMap::invalidateObservation() {
  std_msgs::Header status; status.seq=observation_generation_; status.frame_id=mp_.frame_id_;
  observation_ready_pub_.publish(status); // zero source stamp means NOT READY
}
void SDFMap::observationTimer(const ros::TimerEvent&) {
  const auto next=observation_worker_->snapshot();
  const double pose_age=(ros::Time::now()-observation_odom_stamp_).toSec();
  if(observation_resetting_ || !md_.has_odom_ || observation_odom_stamp_.isZero() ||
     pose_age<0. || pose_age>observation_worker_->maxAge() || !next || next->stamp<=observation_epoch_ ||
     ros::Time::now().toSec()-next->stamp<0. ||
     ros::Time::now().toSec()-next->stamp>observation_worker_->maxAge()) {
    invalidateObservation(); return;
  }
  if(!observation_snapshot_ || next->stamp!=observation_snapshot_->stamp) {
    try {
    const ros::WallTime apply_started=ros::WallTime::now();
    // A gap restarts readiness warmup. A heartbeat is never stamped 'now'.
    if(!observation_snapshot_ || next->stamp-observation_last_>observation_worker_->maxAge())
      observation_first_=next->stamp;
    observation_last_=next->stamp; observation_snapshot_=next;
    pcl::PointCloud<pcl::PointXYZ> cloud; cloud.reserve(next->obstacles.size());
    for(const auto& p:next->obstacles) cloud.push_back(pcl::PointXYZ(p[0],p[1],p[2]));
    cloud.width=uint32_t(cloud.size()); cloud.height=1;
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud,*msg); msg->header.frame_id=mp_.frame_id_;
    msg->header.stamp=ros::Time(next->stamp); msg->header.seq=observation_generation_;
    applying_observation_=true;
    try {cloudCallback(msg);} catch(...) {applying_observation_=false; throw;}
    applying_observation_=false;
    // Compute once per accepted snapshot before exposing it to the FSM/manager.
    updateESDF3d(); md_.esdf_need_update_=false;
    observation_cloud_pub_.publish(msg);
    const auto sample=[](std::deque<double>& values,double v) {
      values.push_back(v); if(values.size()>100) values.pop_front();
    };
    sample(observation_worker_samples_,next->work_ms);
    sample(observation_apply_samples_,(ros::WallTime::now()-apply_started).toSec()*1000.);
    sample(observation_age_samples_,ros::Time::now().toSec()-next->stamp);
    } catch(const std::exception& error) {
      applying_observation_=false; observation_snapshot_.reset();
      invalidateObservation();
      ROS_ERROR_STREAM_THROTTLE(2.,"Observed map rebuild failed: "<<error.what());
      return;
    }
  }
  std_msgs::Header status; status.seq=observation_generation_; status.frame_id=mp_.frame_id_;
  if(md_.has_odom_ && observationCovered(md_.camera_pos_) && getInflateOccupancy(md_.camera_pos_)==0 &&
     observation_last_-observation_first_>=observation_warmup_) status.stamp=ros::Time(observation_last_);
  observation_ready_pub_.publish(status);
  if(observation_diagnostics_at_.isZero() ||
     (ros::WallTime::now()-observation_diagnostics_at_).toSec()>=1.) {
    observation_diagnostics_at_=ros::WallTime::now();
    std::ostringstream json;
    const auto p95=[](const std::deque<double>& values) {
      if(values.empty()) return 0.;
      std::vector<double> sorted(values.begin(),values.end()); std::sort(sorted.begin(),sorted.end());
      return sorted[size_t(std::ceil(.95*sorted.size()))-1];
    };
    json<<"{\"generation\":"<<observation_generation_<<",\"source_stamp\":"<<std::fixed<<next->stamp
        <<",\"source_age\":"<<ros::Time::now().toSec()-next->stamp
        <<",\"worker_ms\":"<<next->work_ms<<",\"input_points\":"<<next->input_points
        <<",\"occupied_voxels\":"<<next->obstacles.size()<<",\"ready\":"
        <<(status.stamp.isZero()?"false":"true")
        <<",\"worker_p95_ms\":"<<p95(observation_worker_samples_)
        <<",\"map_apply_p95_ms\":"<<p95(observation_apply_samples_)
        <<",\"map_age_p95_sec\":"<<p95(observation_age_samples_)
        <<",\"samples\":"<<observation_age_samples_.size()<<"}";
    std_msgs::String diagnostics; diagnostics.data=json.str(); observation_diagnostics_pub_.publish(diagnostics);
  }
}

bool SDFMap::rebuildObservation(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
  const auto ground=[&]() {
    const double a=(ros::Time::now()-observation_fc_received_).toSec();
    const double b=(ros::Time::now()-observation_landed_received_).toSec();
    const double source_a=(ros::Time::now()-observation_fc_.header.stamp).toSec();
    const double source_b=(ros::Time::now()-observation_landed_.header.stamp).toSec();
    return observation_fc_.connected && !observation_fc_.armed &&
        observation_landed_.landed_state==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
        !observation_fc_.header.stamp.isZero() && !observation_landed_.header.stamp.isZero() &&
        a>=0. && a<=.5 && b>=0. && b<=.5 && source_a>=0. && source_a<=.5 && source_b>=0. && source_b<=.5;
  };
  if(!ground()) {response.message="requires fresh connected, disarmed, on-ground FC"; return true;}
  std_srvs::Trigger prepare;
  auto manager=node_.serviceClient<std_srvs::Trigger>("/navigation/prepare_map_rebuild");
  if(!manager.call(prepare) || !prepare.response.success) {
    response.message="mission manager did not admit ground map rebuild"; return true;
  }
  // Start admission stays blocked even if any later operation fails.
  observation_resetting_=true; invalidateObservation();
  node_.setParam("/navigation_observation/rebuilding",true);
  std_srvs::Trigger reset;
  auto freedom=node_.serviceClient<std_srvs::Trigger>("/freedom/reset_map");
  if(!ground() || !freedom.call(reset) || !reset.response.success || !ground()) {
    observation_snapshot_.reset();
    response.message="rebuild incomplete; keep mission inhibited and retry on ground"; return true;
  }
  observation_epoch_=ros::Time::now().toSec();
  observation_worker_->reset(observation_epoch_); observation_snapshot_.reset();
  observation_resetting_=false;
  observation_first_=observation_last_=0.; ++observation_generation_;
  observation_worker_samples_.clear(); observation_apply_samples_.clear(); observation_age_samples_.clear();
  resetBuffer();
  std::fill(md_.occupancy_buffer_.begin(),md_.occupancy_buffer_.end(),mp_.clamp_min_log_-mp_.unknown_flag_);
  std::fill(md_.distance_buffer_all_.begin(),md_.distance_buffer_all_.end(),0.);
  md_.esdf_need_update_=false; md_.occ_need_update_=false;
  invalidateObservation();
  // Keep /rebuilding true through warmup; the mission manager releases it
  // only after a new-generation, fresh READY and no active task.
  response.success=true; response.message="new map epoch started; wait for observation READY";
  return true;
}
