#include <control_manager/output_publisher.h>

namespace pairs_uav_managers
{

namespace control_manager
{

OutputPublisher::OutputPublisher() {
}

OutputPublisher::OutputPublisher(ros::NodeHandle& nh) {

  ph_hw_api_actuator_cmd_              = pairs_lib::PublisherHandler<pairs_msgs::HwApiActuatorCmd>(nh, "hw_api_actuator_cmd_out", 1);
  ph_hw_api_control_group_cmd_         = pairs_lib::PublisherHandler<pairs_msgs::HwApiControlGroupCmd>(nh, "hw_api_control_group_cmd_out", 1);
  ph_hw_api_attitude_rate_cmd_         = pairs_lib::PublisherHandler<pairs_msgs::HwApiAttitudeRateCmd>(nh, "hw_api_attitude_rate_cmd_out", 1);
  ph_hw_api_attitude_cmd_              = pairs_lib::PublisherHandler<pairs_msgs::HwApiAttitudeCmd>(nh, "hw_api_attitude_cmd_out", 1);
  ph_hw_api_acceleration_hdg_rate_cmd_ = pairs_lib::PublisherHandler<pairs_msgs::HwApiAccelerationHdgRateCmd>(nh, "hw_api_acceleration_hdg_rate_cmd_out", 1);
  ph_hw_api_acceleration_hdg_cmd_      = pairs_lib::PublisherHandler<pairs_msgs::HwApiAccelerationHdgCmd>(nh, "hw_api_acceleration_hdg_cmd_out", 1);
  ph_hw_api_velocity_hdg_rate_cmd_     = pairs_lib::PublisherHandler<pairs_msgs::HwApiVelocityHdgRateCmd>(nh, "hw_api_velocity_hdg_rate_cmd_out", 1);
  ph_hw_api_velocity_hdg_cmd_          = pairs_lib::PublisherHandler<pairs_msgs::HwApiVelocityHdgCmd>(nh, "hw_api_velocity_hdg_cmd_out", 1);
  ph_hw_api_position_cmd_              = pairs_lib::PublisherHandler<pairs_msgs::HwApiPositionCmd>(nh, "hw_api_position_cmd_out", 1);
}

void OutputPublisher::publish(const Controller::HwApiOutputVariant& control_output) {

  std::visit(OutputPublisher::PublisherVisitor(this), control_output);
}

// | ------------------------- private ------------------------ |

void OutputPublisher::publish(const pairs_msgs::HwApiActuatorCmd& msg) {
  ph_hw_api_actuator_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiControlGroupCmd& msg) {
  ph_hw_api_control_group_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiAttitudeRateCmd& msg) {
  ph_hw_api_attitude_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiAttitudeCmd& msg) {
  ph_hw_api_attitude_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiAccelerationHdgRateCmd& msg) {
  ph_hw_api_acceleration_hdg_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiAccelerationHdgCmd& msg) {
  ph_hw_api_acceleration_hdg_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiVelocityHdgRateCmd& msg) {
  ph_hw_api_velocity_hdg_rate_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiVelocityHdgCmd& msg) {
  ph_hw_api_velocity_hdg_cmd_.publish(msg);
}

void OutputPublisher::publish(const pairs_msgs::HwApiPositionCmd& msg) {
  ph_hw_api_position_cmd_.publish(msg);
}

}  // namespace control_manager

}  // namespace pairs_uav_managers
