cmake_minimum_required(VERSION 3.28)
project(pairs_uav_managers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use PAIRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[pairs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  pairs_msgs
  pairs_lib
  pluginlib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  pairs_uav_hw_api
  visualization_msgs
  tf2_geometry_msgs
  Eigen3
  pairs_uav_testing
  backward_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  PairsUavManagers_EstimationManager
  PairsUavManagers_TransformManager
  PairsUavManagers_ControlManager
  PairsUavManagers_UavManager
  PairsUavManagers_ConstraintManager
  PairsUavManagers_GainManager
  PairsUavManagers_SafetyAreaManager
)

set(PLUGINS
  PairsUavManagers_NullTracker
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Estimation Manager

add_library(PairsUavManagers_EstimationManager SHARED
  src/estimation_manager/estimation_manager.cpp
  src/estimation_manager/estimator.cpp
  src/estimation_manager/estimators/state_estimator.cpp
  src/estimation_manager/estimators/agl_estimator.cpp
)

target_include_directories(PairsUavManagers_EstimationManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_EstimationManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_EstimationManager "pairs_uav_managers::estimation_manager::EstimationManager")

target_compile_definitions(PairsUavManagers_EstimationManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Transform Manager

add_library(PairsUavManagers_TransformManager SHARED
  src/transform_manager/transform_manager.cpp
)

target_include_directories(PairsUavManagers_TransformManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_TransformManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_TransformManager "pairs_uav_managers::transform_manager::TransformManager")

target_compile_definitions(PairsUavManagers_TransformManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Control Manager

add_library(PairsUavManagers_ControlManager SHARED
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
)

target_include_directories(PairsUavManagers_ControlManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_ControlManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_ControlManager "pairs_uav_managers::control_manager::ControlManager")

target_compile_definitions(PairsUavManagers_ControlManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Safety Area Manager

add_library(PairsUavManagers_SafetyAreaManager SHARED
  src/safety_area_manager.cpp
)

target_link_libraries(PairsUavManagers_SafetyAreaManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_SafetyAreaManager "pairs_uav_managers::safety_area_manager::SafetyAreaManager")

target_compile_definitions(PairsUavManagers_SafetyAreaManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# UAV Manager

add_library(PairsUavManagers_UavManager SHARED
  src/uav_manager.cpp
)

target_include_directories(PairsUavManagers_UavManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_UavManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_UavManager "pairs_uav_managers::uav_manager::UavManager")

target_compile_definitions(PairsUavManagers_UavManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Constraint Manager

add_library(PairsUavManagers_ConstraintManager SHARED
  src/constraint_manager.cpp
)

target_include_directories(PairsUavManagers_ConstraintManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_ConstraintManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_ConstraintManager "pairs_uav_managers::constraint_manager::ConstraintManager")

target_compile_definitions(PairsUavManagers_ConstraintManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Gain Manager

add_library(PairsUavManagers_GainManager SHARED
  src/gain_manager.cpp
)

target_include_directories(PairsUavManagers_GainManager
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_GainManager
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

rclcpp_components_register_nodes(PairsUavManagers_GainManager "pairs_uav_managers::gain_manager::GainManager")

target_compile_definitions(PairsUavManagers_GainManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Null Tracker

add_library(PairsUavManagers_NullTracker SHARED
  src/null_tracker.cpp
)

target_include_directories(PairsUavManagers_NullTracker
  PRIVATE
    "include"
)

target_link_libraries(PairsUavManagers_NullTracker
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    pairs_uav_hw_api::pairs_uav_hw_api
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(PairsUavManagers_NullTracker PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER} COVERAGE=${COVERAGE})

# Public interface of this library

add_library(PairsUavManagers_PairsUavManagers INTERFACE)
add_library(pairs_uav_managers::pairs_uav_managers ALIAS PairsUavManagers_PairsUavManagers)
set_target_properties(PairsUavManagers_PairsUavManagers
  PROPERTIES
    EXPORT_NAME pairs_uav_managers
    OUTPUT_NAME pairs_uav_managers
)

target_sources(PairsUavManagers_PairsUavManagers
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    # include/control_manager
    "include/control_manager/output_publisher.h"
    # include/pairs_uav_managers
    "include/pairs_uav_managers/agl_estimator.h"
    "include/pairs_uav_managers/controller.h"
    "include/pairs_uav_managers/state_estimator.h"
    "include/pairs_uav_managers/tracker.h"
    # include/pairs_uav_managers/control_manager
    "include/pairs_uav_managers/control_manager/common_handlers.h"
    "include/pairs_uav_managers/control_manager/common.h"
    "include/pairs_uav_managers/control_manager/private_handlers.h"
    # include/pairs_uav_managers/estimation_manager
    "include/pairs_uav_managers/estimation_manager/common_handlers.h"
    "include/pairs_uav_managers/estimation_manager/estimator.h"
    "include/pairs_uav_managers/estimation_manager/private_handlers.h"
    "include/pairs_uav_managers/estimation_manager/support.h"
    "include/pairs_uav_managers/estimation_manager/types.h"
    # include/transform_manager
    "include/transform_manager/tf_mapping_origin.h"
    "include/transform_manager/tf_source.h"
)

target_link_libraries(PairsUavManagers_PairsUavManagers
  INTERFACE
    ${LIBRARIES}
    rclcpp::rclcpp
    pairs_lib::pairs_lib
    pluginlib::pluginlib
    Eigen3::Eigen
    ${pairs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${pairs_uav_hw_api_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(pairs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  if(NOT DEFINED ENV{IN_NIX_SHELL})
    message(WARNING "Compiling lint tests")
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint()
  endif()

endif()

## | -------------------- events container -------------------- |

add_executable(events_container src/events_container.cpp)

# Link dependencies
ament_target_dependencies(events_container
  rclcpp
  rclcpp_components
)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    PairsUavManagers_PairsUavManagers
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

# Install the executable
install(TARGETS events_container
  DESTINATION lib/${PROJECT_NAME}
)


install(
  TARGETS
    ${PLUGINS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
