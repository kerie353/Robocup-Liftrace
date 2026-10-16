#ifndef PLAN_ENV_OBSERVATION_WORKER_H
#define PLAN_ENV_OBSERVATION_WORKER_H
#include <plan_env/observation_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace fast_planner {
// One persistent worker, queue size one. Never run PCL/TF/raycasting in the FSM.
class ObservationWorker {
public:
  ObservationWorker(ros::NodeHandle& nh, ObservationGrid::Config config, const std::string& frame);
  ~ObservationWorker();
  void candidates(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void reset(double epoch);
  std::shared_ptr<const ObservationGrid::Snapshot> snapshot();
  double maxAge() const { return max_age_; }
private:
  void submit(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void run();
  ObservationGrid grid_;
  std::string frame_, sensor_frame_;
  double max_age_=.35, clear_period_=.2, update_period_=.1, epoch_=0.;
  size_t max_input_points_=100000;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber sub_;
  std::mutex mutex_, grid_mutex_;
  std::condition_variable wake_;
  sensor_msgs::PointCloud2ConstPtr pending_, candidates_;
  std::shared_ptr<const ObservationGrid::Snapshot> latest_;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
};
}
#endif
