#ifndef CONTROL_MANAGER_COMMON_HANDLERS_H
#define CONTROL_MANAGER_COMMON_HANDLERS_H

#include <pairs_lib/transformer.h>
#include <pairs_lib/scope_timer.h>
#include <pairs_lib/quadratic_throttle_model.h>

namespace pairs_uav_managers
{

namespace control_manager
{

/* safety area handler //{ */

typedef boost::function<bool(const pairs_msgs::ReferenceStamped &point)> isPointInSafetyArea3d_t;
typedef boost::function<bool(const pairs_msgs::ReferenceStamped &point)> isPointInSafetyArea2d_t;
typedef boost::function<double(const std::string &frame_id)>           getMaxZ_t;
typedef boost::function<double(const std::string &frame_id)>           getMinZ_t;

struct SafetyArea_t
{
  pairs_uav_managers::control_manager::isPointInSafetyArea3d_t isPointInSafetyArea3d;
  pairs_uav_managers::control_manager::isPointInSafetyArea2d_t isPointInSafetyArea2d;
  pairs_uav_managers::control_manager::getMaxZ_t               getMaxZ;
  pairs_uav_managers::control_manager::getMinZ_t               getMinZ;
  bool                                                       use_safety_area;
};

//}

/* scope timer handler //{ */

struct ScopeTimer_t
{
  bool                                       enabled;
  std::shared_ptr<pairs_lib::ScopeTimerLogger> logger;
};

//}

typedef boost::function<double(void)> getMass_t;

struct DetailedModelParams_t
{
  Eigen::MatrixXd control_group_mixer;
  Eigen::MatrixXd force_torque_mixer;
  Eigen::Matrix3d inertia;
  double          prop_radius;
  double          arm_length;
  double          body_height;
};

struct ControlOutputModalities_t
{
  bool actuators             = false;
  bool control_group         = false;
  bool attitude_rate         = false;
  bool attitude              = false;
  bool acceleration_hdg_rate = false;
  bool acceleration_hdg      = false;
  bool velocity_hdg_rate     = false;
  bool velocity_hdg          = false;
  bool position              = false;
};

struct CommonHandlers_t
{
  SafetyArea_t                                      safety_area;
  std::shared_ptr<pairs_lib::Transformer>             transformer;
  ScopeTimer_t                                      scope_timer;
  getMass_t                                         getMass;
  double                                            g;
  pairs_lib::quadratic_throttle_model::motor_params_t throttle_model;
  std::optional<DetailedModelParams_t>              detailed_model_params;
  ControlOutputModalities_t                         control_output_modalities;
  std::string                                       uav_name;
  ros::NodeHandle                                   parent_nh;
};

}  // namespace control_manager

}  // namespace pairs_uav_managers

#endif  // CONTROL_MANAGER_COMMON_HANDLERS_H
