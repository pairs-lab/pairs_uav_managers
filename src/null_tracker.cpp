#include <ros/ros.h>

#include <pairs_uav_managers/tracker.h>

namespace pairs_uav_managers
{

/* //{ class NullTracker */

class NullTracker : public pairs_uav_managers::Tracker {

public:
  ~NullTracker(){};

  bool initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  std::tuple<bool, std::string> activate([[maybe_unused]] const std::optional<pairs_msgs::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<pairs_msgs::TrackerCommand>   update(const pairs_msgs::UavState &uav_state, const Controller::ControlOutput &last_control_output);
  const pairs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const pairs_msgs::UavState &new_uav_state);

  const pairs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const pairs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const pairs_msgs::VelocityReferenceSrvResponse::ConstPtr   setVelocityReference(const pairs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const pairs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const pairs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

  const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;
  bool            is_active         = false;
  bool            is_initialized    = false;
  bool            callbacks_enabled = false;

  std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers;
};

//}

// | ------------------- trackers interface ------------------- |

/* //{ initialize() */

bool NullTracker::initialize(const ros::NodeHandle &                                                                parent_nh,
                             [[maybe_unused]] std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t>  common_handlers,
                             [[maybe_unused]] std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  ros::NodeHandle nh_(parent_nh, "null_tracker");

  ros::Time::waitForValid();

  is_initialized = true;

  this->common_handlers = common_handlers;

  ROS_INFO("[NullTracker]: initialized");

  return true;
}

//}

/* //{ activate() */

std::tuple<bool, std::string> NullTracker::activate([[maybe_unused]] const std::optional<pairs_msgs::TrackerCommand> &last_tracker_cmd) {

  std::stringstream ss;
  ss << "activated";

  ROS_INFO_STREAM("[NullTracker]: " << ss.str());
  is_active = true;

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void NullTracker::deactivate(void) {

  ROS_INFO("[NullTracker]: deactivated");
  is_active = false;
}

//}

/* //{ resetStatic() */

bool NullTracker::resetStatic(void) {
  return false;
}

//}

/* switchOdometrySource() //{ */

const std_srvs::TriggerResponse::ConstPtr NullTracker::switchOdometrySource([[maybe_unused]] const pairs_msgs::UavState &new_uav_state) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ update() */

std::optional<pairs_msgs::TrackerCommand> NullTracker::update([[maybe_unused]] const pairs_msgs::UavState &       uav_state,
                                                            [[maybe_unused]] const Controller::ControlOutput &last_control_output) {

  return {};
}

//}

/* //{ getStatus() */

const pairs_msgs::TrackerStatus NullTracker::getStatus() {

  pairs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active;
  tracker_status.callbacks_enabled = callbacks_enabled;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr NullTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  std::stringstream ss;

  if (cmd->data != callbacks_enabled) {

    callbacks_enabled = cmd->data;

    ss << "callbacks " << (callbacks_enabled ? "enabled" : "disabled");

    ROS_DEBUG_STREAM("[NullTracker]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled ? "enabled" : "disabled");
  }

  res.message = ss.str();
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(std::make_unique<std_srvs::SetBoolResponse>(res));
}

//}

/* //{ setReference() */

const pairs_msgs::ReferenceSrvResponse::ConstPtr NullTracker::setReference([[maybe_unused]] const pairs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  return pairs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ setVelocityReference() */

const pairs_msgs::VelocityReferenceSrvResponse::ConstPtr NullTracker::setVelocityReference([
    [maybe_unused]] const pairs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return pairs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const pairs_msgs::TrajectoryReferenceSrvResponse::ConstPtr NullTracker::setTrajectoryReference([
    [maybe_unused]] const pairs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return pairs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

// | --------------------- other services --------------------- |

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr NullTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr NullTracker::setConstraints([
    [maybe_unused]] const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  return pairs_msgs::DynamicsConstraintsSrvResponse::Ptr();
}

//}

}  // namespace pairs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_managers::NullTracker, pairs_uav_managers::Tracker)
