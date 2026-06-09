#ifndef PAIRS_UAV_MANAGERS_AGL_ESTIMATOR_H
#define PAIRS_UAV_MANAGERS_AGL_ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <pairs_msgs/UavState.h>
#include <pairs_msgs/Float64Stamped.h>
#include <pairs_msgs/Float64ArrayStamped.h>
#include <pairs_msgs/HwApiCapabilities.h>

#include <pairs_uav_managers/estimation_manager/estimator.h>
#include <pairs_uav_managers/estimation_manager/support.h>

//}

namespace pairs_uav_managers
{

namespace agl
{
const char type[] = "AGL";
}

using namespace estimation_manager;

class AglEstimator : public Estimator {

protected:
  const std::string package_name_ = "pairs_uav_state_estimators";

  ros::NodeHandle nh_;

  pairs_msgs::Float64Stamped agl_height_;
  pairs_msgs::Float64Stamped agl_height_init_;
  mutable std::mutex       mtx_agl_height_;

  pairs_msgs::Float64ArrayStamped agl_height_cov_;
  pairs_msgs::Float64ArrayStamped agl_height_cov_init_;
  mutable std::mutex            mtx_agl_height_cov_;

  bool is_override_frame_id_ = false;

protected:
  mutable pairs_lib::PublisherHandler<pairs_msgs::Float64Stamped>      ph_agl_height_;
  mutable pairs_lib::PublisherHandler<pairs_msgs::Float64ArrayStamped> ph_agl_height_cov_;

public:
  AglEstimator(const std::string &name, const std::string &frame_id, const std::string &package_name)
      : Estimator(agl::type, name, frame_id), package_name_(package_name) {
  }

  virtual ~AglEstimator(void) {
  }

  // virtual methods
  virtual pairs_msgs::Float64Stamped getUavAglHeight() const     = 0;
  virtual std::vector<double>      getHeightCovariance() const = 0;

  // implemented methods
  void publishAglHeight() const;
  void publishCovariance() const;
  bool isCompatibleWithHwApi(const pairs_msgs::HwApiCapabilitiesConstPtr &hw_api_capabilities) const;
};

}  // namespace pairs_uav_managers

#endif  // PAIRS_UAV_MANAGERS_AGL_ESTIMATOR_H
