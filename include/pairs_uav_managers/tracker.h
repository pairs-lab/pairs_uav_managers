#ifndef PAIRS_UAV_TRACKER_H
#define PAIRS_UAV_TRACKER_H

/* includes //{ */

#include <ros/ros.h>

#include <pairs_uav_managers/control_manager/common_handlers.h>
#include <pairs_uav_managers/control_manager/private_handlers.h>

#include <pairs_msgs/TrackerCommand.h>
#include <pairs_msgs/TrackerStatus.h>
#include <pairs_msgs/UavState.h>

#include <pairs_msgs/Float64Srv.h>
#include <pairs_msgs/Float64SrvRequest.h>
#include <pairs_msgs/Float64SrvResponse.h>

#include <pairs_msgs/Float64.h>

#include <pairs_msgs/ReferenceSrv.h>
#include <pairs_msgs/ReferenceSrvRequest.h>
#include <pairs_msgs/ReferenceSrvResponse.h>

#include <pairs_msgs/VelocityReferenceSrv.h>
#include <pairs_msgs/VelocityReferenceSrvRequest.h>
#include <pairs_msgs/VelocityReferenceSrvResponse.h>

#include <pairs_msgs/TrajectoryReferenceSrv.h>
#include <pairs_msgs/TrajectoryReferenceSrvRequest.h>
#include <pairs_msgs/TrajectoryReferenceSrvResponse.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/SetBoolResponse.h>

#include <pairs_msgs/DynamicsConstraintsSrv.h>
#include <pairs_msgs/DynamicsConstraintsSrvRequest.h>
#include <pairs_msgs/DynamicsConstraintsSrvResponse.h>

#include <pairs_uav_managers/controller.h>

//}

namespace pairs_uav_managers
{

class Tracker {

public:
  /**
   * @brief It is called once for every tracker. The runtime is not limited.
   *
   * @param nh the node handle of the ControlManager
   * @param uav_name the UAV name (e.g., "uav1")
   * @param common_handlers handlers shared between trackers and controllers
   * @param private_handlers handlers provided individually to each tracker
   *
   * @return true if success
   */
  virtual bool initialize(const ros::NodeHandle &nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                          std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) = 0;

  /**
   * @brief It is called before the trackers output will be required and used. Should not take much time (within miliseconds).
   *
   * @param last_tracker_cmd the last command produced by the last active tracker. Should be used as an initial condition to maintain a smooth reference.
   *
   * @return true if success and message
   */
  virtual std::tuple<bool, std::string> activate(const std::optional<pairs_msgs::TrackerCommand> &last_tracker_cmd) = 0;

  /**
   * @brief is called when this trackers output is no longer needed. However, it can be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief It is called during every switch of reference frames of the UAV state estimate.
   * The tracker should recalculate its internal states from old the frame to the new one.
   *
   * @param new_uav_state the new UavState which will come in the next update()
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const pairs_msgs::UavState &new_uav_state) = 0;

  /**
   * @brief Request for reseting the tracker's states given the UAV is static.
   *
   * @return true if success
   */
  virtual bool resetStatic(void) = 0;

  /**
   * @brief The most important routine. It is called with every state estimator update and it should produce a new reference for the controllers.
   *        The run time should be as short as possible (<= 1 ms).
   *
   * @param uav_state the latest UAV state estimate
   * @param last_attitude_cmd the last controller's output command (may be useful)
   *
   * @return the new reference for the controllers
   */
  virtual std::optional<pairs_msgs::TrackerCommand> update(const pairs_msgs::UavState &uav_state, const Controller::ControlOutput &last_control_output) = 0;

  /**
   * @brief A request for the tracker's status.
   *
   * @return the tracker's status
   */
  virtual const pairs_msgs::TrackerStatus getStatus() = 0;

  /**
   * @brief Request for a flight to a given coordinates.
   *
   * @param cmd the reference
   *
   * @return a service response
   */
  virtual const pairs_msgs::ReferenceSrvResponse::ConstPtr setReference(const pairs_msgs::ReferenceSrvRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request for desired velocity reference
   *
   * @param cmd the reference
   *
   * @return a service response
   */
  virtual const pairs_msgs::VelocityReferenceSrvResponse::ConstPtr setVelocityReference(const pairs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request for a flight along a given trajectory
   *
   * @param cmd the reference trajectory
   *
   * @return a service response
   */
  virtual const pairs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const pairs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request for stopping the motion of the UAV.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request to goto to the first trajectory coordinate.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request to start tracking of the pre-loaded trajectory
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request to stop tracking of the pre-loaded trajectory. The hover() routine will be engaged, thus it should be implemented by the tracker.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request to resume the previously stopped trajectory tracking.
   *
   * @param trigger service request
   *
   * @return a service response
   */
  virtual const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request for enabling/disabling callbacks.
   *
   * @param cmd service request
   *
   * @return a service response
   */
  virtual const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) = 0;

  /**
   * @brief Request for setting new constraints.
   *
   * @param constraints to be set
   *
   * @return a service response
   */
  virtual const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) = 0;

  virtual ~Tracker() = default;
};

}  // namespace pairs_uav_managers

#endif
