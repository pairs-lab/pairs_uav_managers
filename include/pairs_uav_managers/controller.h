#ifndef PAIRS_UAV_CONTROLLER_H
#define PAIRS_UAV_CONTROLLER_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <pairs_uav_managers/control_manager/common_handlers.h>
#include <pairs_uav_managers/control_manager/private_handlers.h>

#include <pairs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <pairs_msgs/msg/hw_api_control_group_cmd.hpp>
#include <pairs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_attitude_cmd.hpp>
#include <pairs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_acceleration_hdg_cmd.hpp>
#include <pairs_msgs/msg/hw_api_velocity_hdg_rate_cmd.hpp>
#include <pairs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>
#include <pairs_msgs/msg/hw_api_position_cmd.hpp>

#include <pairs_msgs/msg/controller_diagnostics.hpp>
#include <pairs_msgs/msg/controller_status.hpp>
#include <pairs_msgs/msg/tracker_command.hpp>
#include <pairs_msgs/msg/uav_state.hpp>

#include <pairs_msgs/srv/dynamics_constraints_srv.hpp>

//}

namespace pairs_uav_managers
{

class Controller {
public:
  typedef std::variant<pairs_msgs::msg::HwApiActuatorCmd, pairs_msgs::msg::HwApiControlGroupCmd, pairs_msgs::msg::HwApiAttitudeRateCmd,
                       pairs_msgs::msg::HwApiAttitudeCmd, pairs_msgs::msg::HwApiAccelerationHdgRateCmd, pairs_msgs::msg::HwApiAccelerationHdgCmd,
                       pairs_msgs::msg::HwApiVelocityHdgRateCmd, pairs_msgs::msg::HwApiVelocityHdgCmd, pairs_msgs::msg::HwApiPositionCmd>
      HwApiOutputVariant;

  typedef struct
  {
    std::optional<HwApiOutputVariant>    control_output;
    pairs_msgs::msg::ControllerDiagnostics diagnostics;

    /**
     * @brief Desired orientation is used for checking the orientation control error.
     *        This variable is optional, fill it in if you know it.
     */
    std::optional<Eigen::Quaterniond> desired_orientation;

    /**
     * @brief Desired unbiased acceleration is used by the PAIRS odometry as control input.
     *        This variable is optional, fill it in if you know it.
     */
    std::optional<Eigen::Vector3d> desired_unbiased_acceleration;

    /**
     * @brief Desired heading rate caused by the controllers control action.
     *        This variable is optional, fill it in if you know it.
     */
    std::optional<double> desired_heading_rate;
  } ControlOutput;

  /**
   * @brief Initializes the controller. It is called once for every controller. The runtime is not limited.
   *
   * @param node the node handle of the ControlManager
   * @param name of the controller for distinguishing multiple running instances of the same code
   * @param name_space the parameter namespace of the controller, can be used during initialization of the private node handle
   * @param common_handlers handlers shared between trackers and controllers
   * @param private_handlers handlers provided individually to each controller
   *
   * @return true if success
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                          std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) = 0;

  virtual void destroy(void) = 0;

  /**
   * @brief It is called before the controller output will be required and used. Should not take much time (within miliseconds).
   *
   * @param last_attitude_cmd the last command produced by the last active controller. Should be used as an initial condition, e.g., for re-initializing
   * integrators and estimators.
   *
   * @return true if success
   */
  virtual bool activate(const ControlOutput &last_control_output) = 0;

  /**
   * @brief is called when this controller's output is no longer needed. However, it can be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief It may be called to reset the controllers disturbance estimators.
   */
  virtual void resetDisturbanceEstimators(void) = 0;

  /**
   * @brief This method is called in the main feedback control loop when your controller is NOT active. You can use this to validate your results without
   * endangering the drone. The method is called even before the flight with just the uav_state being supplied.
   *
   * @param uav_state current estimated state of the UAV dynamics
   * @param tracker_command current required control reference (is optional)
   */
  virtual void updateInactive(const pairs_msgs::msg::UavState &uav_state, const std::optional<pairs_msgs::msg::TrackerCommand> &tracker_command) = 0;

  /**
   * @brief This method is called in the main feedback control loop when your controller IS active and when it is supposed to produce a control output.
   *
   * @param uav_state current estimated state of the UAV dynamics
   * @param tracker_command current required control reference
   *
   * @return produced control output
   */
  virtual ControlOutput updateActive(const pairs_msgs::msg::UavState &uav_state, const pairs_msgs::msg::TrackerCommand &tracker_command) = 0;

  /**
   * @brief A request for the controller's status.
   *
   * @return the controller's status
   */
  virtual const pairs_msgs::msg::ControllerStatus getStatus() = 0;

  /**
   * @brief It is called during every switch of reference frames of the UAV state estimate.
   * The controller should recalculate its internal states from old the frame to the new one.
   *
   * @param new_uav_state the new UavState which will come in the next update()
   */
  virtual void switchOdometrySource(const pairs_msgs::msg::UavState &new_uav_state) = 0;

  /**
   * @brief Request for setting new constraints.
   *
   * @param constraints to be set
   *
   * @return a service response
   */
  virtual const std::shared_ptr<pairs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<pairs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) = 0;

  virtual ~Controller() = default;
};

} // namespace pairs_uav_managers

#endif
