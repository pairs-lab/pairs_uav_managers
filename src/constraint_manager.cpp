/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

#include <pairs_msgs/ConstraintManagerDiagnostics.h>
#include <pairs_msgs/EstimationDiagnostics.h>
#include <pairs_msgs/DynamicsConstraintsSrv.h>
#include <pairs_msgs/DynamicsConstraintsSrvRequest.h>
#include <pairs_msgs/String.h>
#include <pairs_msgs/ConstraintsOverride.h>

#include <pairs_lib/profiler.h>
#include <pairs_lib/scope_timer.h>
#include <pairs_lib/param_loader.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/service_client_handler.h>
#include <pairs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//}

namespace pairs_uav_managers
{

namespace constraint_manager
{

/* //{ class ConstraintManager */

class ConstraintManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  std::atomic<bool> is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  std::vector<std::string> _estimator_type_names_;

  std::vector<std::string>                                       _constraint_names_;
  std::map<std::string, pairs_msgs::DynamicsConstraintsSrvRequest> _constraints_;

  std::map<std::string, std::vector<std::string>> _map_type_allowed_constraints_;
  std::map<std::string, std::string>              _map_type_default_constraints_;

  // | --------------------- service clients -------------------- |

  pairs_lib::ServiceClientHandler<pairs_msgs::DynamicsConstraintsSrv> sc_set_constraints_;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscribeHandler<pairs_msgs::EstimationDiagnostics> sh_estimation_diag_;

  // | ------------- constraint management ------------- |

  bool setConstraints(std::string constraints_names);

  ros::ServiceServer service_server_set_constraints_;
  bool               callbackSetConstraints(pairs_msgs::String::Request& req, pairs_msgs::String::Response& res);

  std::string last_estimator_name_;
  std::mutex  mutex_last_estimator_name_;

  void       timerConstraintManagement(const ros::TimerEvent& event);
  ros::Timer timer_constraint_management_;
  double     _constraint_management_rate_;

  std::string current_constraints_;
  std::mutex  mutex_current_constraints_;

  // | ------------------ constraints override ------------------ |

  ros::ServiceServer service_server_constraints_override_;
  bool               callbackConstraintsOverride(pairs_msgs::ConstraintsOverride::Request& req, pairs_msgs::ConstraintsOverride::Response& res);

  std::atomic<bool>                    override_constraints_         = false;
  std::atomic<bool>                    constraints_override_updated_ = false;
  std::mutex                           mutex_constraints_override_;
  pairs_msgs::ConstraintsOverrideRequest constraints_override_;

  // | ------------------ diagnostics publisher ----------------- |

  pairs_lib::PublisherHandler<pairs_msgs::ConstraintManagerDiagnostics> ph_diagnostics_;

  void       timerDiagnostics(const ros::TimerEvent& event);
  ros::Timer timer_diagnostics_;
  double     _diagnostics_rate_;

  // | ------------------------ profiler ------------------------ |

  pairs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<pairs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | ------------------------- helpers ------------------------ |

  bool stringInVector(const std::string& value, const std::vector<std::string>& vector);
};

//}

/* //{ onInit() */

void ConstraintManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[ConstraintManager]: initializing");

  // | ------------------------- params ------------------------- |

  pairs_lib::ParamLoader param_loader(nh_, "ConstraintManager");

  std::string custom_config_path;
  std::string platform_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("platform_config", platform_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");
  param_loader.addYamlFileFromParam("public_constraints");

  const std::string yaml_prefix = "pairs_uav_managers/constraint_manager/";

  // params passed from the launch file are not prefixed
  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam(yaml_prefix + "constraints", _constraint_names_);

  param_loader.loadParam(yaml_prefix + "estimator_types", _estimator_type_names_);

  param_loader.loadParam(yaml_prefix + "rate", _constraint_management_rate_);
  param_loader.loadParam(yaml_prefix + "diagnostics_rate", _diagnostics_rate_);

  std::vector<std::string>::iterator it;

  // loading constraint names
  for (it = _constraint_names_.begin(); it != _constraint_names_.end(); ++it) {

    ROS_INFO_STREAM("[ConstraintManager]: loading constraints '" << *it << "'");

    pairs_msgs::DynamicsConstraintsSrvRequest new_constraints;

    param_loader.loadParam(yaml_prefix + *it + "/horizontal/speed", new_constraints.constraints.horizontal_speed);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/acceleration", new_constraints.constraints.horizontal_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/jerk", new_constraints.constraints.horizontal_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/horizontal/snap", new_constraints.constraints.horizontal_snap);

    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/speed", new_constraints.constraints.vertical_ascending_speed);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/acceleration", new_constraints.constraints.vertical_ascending_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/jerk", new_constraints.constraints.vertical_ascending_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/ascending/snap", new_constraints.constraints.vertical_ascending_snap);

    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/speed", new_constraints.constraints.vertical_descending_speed);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/acceleration", new_constraints.constraints.vertical_descending_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/jerk", new_constraints.constraints.vertical_descending_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/vertical/descending/snap", new_constraints.constraints.vertical_descending_snap);

    param_loader.loadParam(yaml_prefix + *it + "/heading/speed", new_constraints.constraints.heading_speed);
    param_loader.loadParam(yaml_prefix + *it + "/heading/acceleration", new_constraints.constraints.heading_acceleration);
    param_loader.loadParam(yaml_prefix + *it + "/heading/jerk", new_constraints.constraints.heading_jerk);
    param_loader.loadParam(yaml_prefix + *it + "/heading/snap", new_constraints.constraints.heading_snap);

    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/roll", new_constraints.constraints.roll_rate);
    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/pitch", new_constraints.constraints.pitch_rate);
    param_loader.loadParam(yaml_prefix + *it + "/angular_speed/yaw", new_constraints.constraints.yaw_rate);

    double tilt_deg;

    param_loader.loadParam(yaml_prefix + *it + "/tilt", tilt_deg);

    new_constraints.constraints.tilt = M_PI * (tilt_deg / 180.0);

    _constraints_.insert(std::pair<std::string, pairs_msgs::DynamicsConstraintsSrvRequest>(*it, new_constraints));
  }

  // loading the allowed constraints lists
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam(yaml_prefix + "allowed_constraints/" + *it, temp_vector);

    std::vector<std::string>::iterator it2;
    for (it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _constraint_names_)) {
        ROS_ERROR("[ConstraintManager]: the element '%s' of %s/allowed_constraints is not a valid constraint!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    _map_type_allowed_constraints_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // loading the default constraints
  for (it = _estimator_type_names_.begin(); it != _estimator_type_names_.end(); ++it) {

    std::string temp_str;
    param_loader.loadParam(yaml_prefix + "default_constraints/" + *it, temp_str);

    if (!stringInVector(temp_str, _map_type_allowed_constraints_.at(*it))) {
      ROS_ERROR("[ConstraintManager]: the element '%s' of %s/allowed_constraints is not a valid constraint!", temp_str.c_str(), it->c_str());
      ros::shutdown();
    }

    _map_type_default_constraints_.insert(std::pair<std::string, std::string>(*it, temp_str));
  }

  ROS_INFO("[ConstraintManager]: done loading dynamic params");

  current_constraints_ = "";
  last_estimator_name_ = "";

  // | ------------------------ services ------------------------ |

  service_server_set_constraints_ = nh_.advertiseService("set_constraints_in", &ConstraintManager::callbackSetConstraints, this);

  service_server_constraints_override_ = nh_.advertiseService("constraints_override_in", &ConstraintManager::callbackConstraintsOverride, this);

  sc_set_constraints_ = pairs_lib::ServiceClientHandler<pairs_msgs::DynamicsConstraintsSrv>(nh_, "set_constraints_out");

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ConstraintManager";
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_estimation_diag_ = pairs_lib::SubscribeHandler<pairs_msgs::EstimationDiagnostics>(shopts, "estimation_diagnostics_in");

  // | ----------------------- publishers ----------------------- |

  ph_diagnostics_ = pairs_lib::PublisherHandler<pairs_msgs::ConstraintManagerDiagnostics>(nh_, "diagnostics_out", 10);

  // | ------------------------- timers ------------------------- |

  timer_constraint_management_ = nh_.createTimer(ros::Rate(_constraint_management_rate_), &ConstraintManager::timerConstraintManagement, this);
  timer_diagnostics_           = nh_.createTimer(ros::Rate(_diagnostics_rate_), &ConstraintManager::timerDiagnostics, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = pairs_lib::Profiler(nh_, "ConstraintManager", _profiler_enabled_);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<pairs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ConstraintManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[ConstraintManager]: initialized");

  ROS_DEBUG("[ConstraintManager]: debug output is enabled");
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setConstraints() //{ */

bool ConstraintManager::setConstraints(std::string constraints_name) {

  std::map<std::string, pairs_msgs::DynamicsConstraintsSrvRequest>::iterator it;
  it = _constraints_.find(constraints_name);

  if (it == _constraints_.end()) {
    ROS_ERROR("[ConstraintManager]: could not setConstraints(), the constraint name '%s' is not on the list", constraints_name.c_str());
    return false;
  }

  pairs_msgs::DynamicsConstraintsSrv srv_call;

  srv_call.request = it->second;

  if (override_constraints_) {

    auto constraints_override = pairs_lib::get_mutexed(mutex_constraints_override_, constraints_override_);

    if (constraints_override.acceleration_horizontal > 0 &&
        constraints_override.acceleration_horizontal <= srv_call.request.constraints.horizontal_acceleration) {
      srv_call.request.constraints.horizontal_acceleration = constraints_override.acceleration_horizontal;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[ConstraintManager]: required horizontal acceleration override is out of bounds");
    }

    if (constraints_override.acceleration_vertical > 0 &&
        constraints_override.acceleration_vertical <= srv_call.request.constraints.vertical_ascending_acceleration &&
        constraints_override.acceleration_vertical <= srv_call.request.constraints.vertical_descending_acceleration) {
      srv_call.request.constraints.vertical_ascending_acceleration  = constraints_override.acceleration_vertical;
      srv_call.request.constraints.vertical_descending_acceleration = constraints_override.acceleration_vertical;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[ConstraintManager]: required vertical acceleration override is out of bounds");
    }
  }

  bool res = sc_set_constraints_.call(srv_call);

  if (!res) {

    ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the service for setting constraints has failed!");
    return false;

  } else {

    if (srv_call.response.success) {

      pairs_lib::set_mutexed(mutex_current_constraints_, constraints_name, current_constraints_);
      return true;

    } else {

      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: set service for setting constraints returned: '%s'", srv_call.response.message.c_str());
      return false;
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | -------------------- service callbacks ------------------- |

/* //{ callbackSetConstraints() */

bool ConstraintManager::callbackSetConstraints(pairs_msgs::String::Request& req, pairs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (!sh_estimation_diag_.hasMsg()) {

    ss << "missing odometry diagnostics";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  if (!stringInVector(req.value, _constraint_names_)) {

    ss << "the constraints '" << req.value.c_str() << "' do not exist (in the ConstraintManager's config)";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!stringInVector(req.value, _map_type_allowed_constraints_.at(estimation_diagnostics->current_state_estimator))) {

    ss << "the constraints '" << req.value.c_str() << "' are not allowed given the current odometry type";

    ROS_WARN_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  override_constraints_ = false;

  // try to set the constraints
  if (!setConstraints(req.value)) {

    ss << "the ControlManager could not set the constraints";

    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;

  } else {

    ss << "the constraints '" << req.value.c_str() << "' were set";

    ROS_INFO_STREAM_THROTTLE(1.0, "[ConstraintManager]: " << ss.str());

    res.message = ss.str();
    res.success = true;
    return true;
  }
}

//}

/* callackConstraintsOverride() //{ */

bool ConstraintManager::callbackConstraintsOverride(pairs_msgs::ConstraintsOverride::Request& req, pairs_msgs::ConstraintsOverride::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_constraints_override_);

    constraints_override_ = req;
  }

  override_constraints_         = true;
  constraints_override_updated_ = true;

  ROS_INFO_THROTTLE(0.1, "[ConstraintManager]: setting constraints override");

  res.message = "override set";
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerConstraintManagement() //{ */

void ConstraintManager::timerConstraintManagement(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  pairs_lib::Routine    profiler_routine = profiler_.createRoutine("timerConstraintManagement", _constraint_management_rate_, 0.01, event);
  pairs_lib::ScopeTimer timer            = pairs_lib::ScopeTimer("ContraintManager::timerConstraintManagement", scope_timer_logger_, scope_timer_enabled_);

  auto current_constraints = pairs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);
  auto last_estimator_name = pairs_lib::get_mutexed(mutex_last_estimator_name_, last_estimator_name_);

  if (!sh_estimation_diag_.hasMsg()) {
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  // | --- automatically set constraints when the state estimator changes -- |
  if (estimation_diagnostics->current_state_estimator != last_estimator_name_) {

    ROS_INFO_THROTTLE(1.0, "[ConstraintManager]: the state estimator has changed! %s -> %s", last_estimator_name_.c_str(),
                      estimation_diagnostics->current_state_estimator.c_str());

    std::map<std::string, std::string>::iterator it;
    it = _map_type_default_constraints_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_default_constraints_.end()) {

      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the state estimator type '%s' was not specified in the constraint_manager's config!",
                        estimation_diagnostics->current_state_estimator.c_str());

    } else {

      // if the current constraints are within the allowed state estimator types, do nothing
      if (stringInVector(current_constraints, _map_type_allowed_constraints_.at(estimation_diagnostics->current_state_estimator))) {

        last_estimator_name = estimation_diagnostics->current_state_estimator;

        // else, try to set the initial constraints
      } else {

        ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the current constraints '%s' are not within the allowed constraints for '%s'", current_constraints.c_str(),
                          estimation_diagnostics->current_state_estimator.c_str());

        if (setConstraints(it->second)) {

          last_estimator_name = estimation_diagnostics->current_state_estimator;

          ROS_INFO_THROTTLE(1.0, "[ConstraintManager]: constraints set to initial: '%s'", it->second.c_str());

        } else {

          ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: could not set constraints!");
        }
      }
    }
  }

  if (constraints_override_updated_) {

    std::map<std::string, std::string>::iterator it;
    it = _map_type_default_constraints_.find(last_estimator_name_);

    ROS_INFO_THROTTLE(0.1, "[ConstraintManager]: re-setting constraints with user value override");

    if (setConstraints(it->second)) {
      constraints_override_updated_ = false;
    } else {
      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: could not re-set the constraints!");
    }
  }

  pairs_lib::set_mutexed(mutex_last_estimator_name_, last_estimator_name, last_estimator_name_);
}

//}

/* timerDiagnostics() //{ */

void ConstraintManager::timerDiagnostics(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  pairs_lib::Routine    profiler_routine = profiler_.createRoutine("timerDiagnostics", _diagnostics_rate_, 0.01, event);
  pairs_lib::ScopeTimer timer            = pairs_lib::ScopeTimer("ContraintManager::timerDiagnostics", scope_timer_logger_, scope_timer_enabled_);

  if (!sh_estimation_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(10.0, "[ConstraintManager]: can not do constraint management, missing estimation diagnostics!");
    return;
  }

  auto estimation_diagnostics = sh_estimation_diag_.getMsg();

  auto current_constraints = pairs_lib::get_mutexed(mutex_current_constraints_, current_constraints_);

  if (current_constraints == "") {  // this could happend just before timerConstraintManagement() finishes
    return;
  }

  pairs_msgs::ConstraintManagerDiagnostics diagnostics;

  diagnostics.stamp        = ros::Time::now();
  diagnostics.current_name = current_constraints;
  diagnostics.loaded       = _constraint_names_;

  // get the available constraints
  {
    std::map<std::string, std::vector<std::string>>::iterator it;
    it = _map_type_allowed_constraints_.find(estimation_diagnostics->current_state_estimator);

    if (it == _map_type_allowed_constraints_.end()) {
      ROS_WARN_THROTTLE(1.0, "[ConstraintManager]: the state estimator '%s' was not specified in the constraint_manager's config!",
                        estimation_diagnostics->current_state_estimator.c_str());
    } else {
      diagnostics.available = it->second;
    }
  }

  // get the current constraint values
  {
    std::map<std::string, pairs_msgs::DynamicsConstraintsSrvRequest>::iterator it;
    it = _constraints_.find(current_constraints);

    if (it == _constraints_.end()) {
      ROS_ERROR("[ConstraintManager]: current constraints '%s' not found in the constraint list!", current_constraints.c_str());
      return;
    }

    diagnostics.current_values = it->second.constraints;
  }

  ph_diagnostics_.publish(diagnostics);
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* stringInVector() //{ */

bool ConstraintManager::stringInVector(const std::string& value, const std::vector<std::string>& vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

}  // namespace constraint_manager

}  // namespace pairs_uav_managers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_managers::constraint_manager::ConstraintManager, nodelet::Nodelet)
