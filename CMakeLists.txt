cmake_minimum_required(VERSION 3.5)
project(pairs_uav_managers)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  nodelet
  geometry_msgs
  std_msgs
  nav_msgs
  pairs_msgs
  visualization_msgs
  pairs_lib
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(LIBRARIES
  PairsUavManagers_GainManager
  PairsUavManagers_ConstraintManager
  PairsUavManagers_ControlManager
  PairsUavManagers_UavManager
  PairsUavManagers_EstimationManager
  PairsUavManagers_TransformManager
  PairsUavManagers_NullTracker
  )

catkin_package(
  INCLUDE_DIRS include test/include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include test/include
  ${catkin_INCLUDE_DIRS}
  )

# PairsUavManagers_ControlManager

add_library(PairsUavManagers_ControlManager
  src/control_manager/control_manager.cpp
  src/control_manager/output_publisher.cpp
  src/control_manager/common/common.cpp
  )

add_dependencies(PairsUavManagers_ControlManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_ControlManager
  ${catkin_LIBRARIES}
  )

# PairsUavManagers_NullTracker

add_library(PairsUavManagers_NullTracker
  src/null_tracker.cpp
  )

add_dependencies(PairsUavManagers_NullTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_NullTracker
  ${catkin_LIBRARIES}
  )

# PairsUavManagers_UavManager

add_library(PairsUavManagers_UavManager
  src/uav_manager.cpp
  )

add_dependencies(PairsUavManagers_UavManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_UavManager
  ${catkin_LIBRARIES}
  )

# PairsUavManagers_GainManager

add_library(PairsUavManagers_GainManager
  src/gain_manager.cpp
  )

add_dependencies(PairsUavManagers_GainManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_GainManager
  ${catkin_LIBRARIES}
  )

# PairsUavManagers_ConstraintManager

add_library(PairsUavManagers_ConstraintManager
  src/constraint_manager.cpp
  )

add_dependencies(PairsUavManagers_ConstraintManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_ConstraintManager
  ${catkin_LIBRARIES}
  )

# PairsUavManagers_EstimationManager
add_library(PairsUavManagers_EstimationManager
  src/estimation_manager/estimation_manager.cpp
  src/estimation_manager/estimator.cpp
  src/estimation_manager/estimators/state_estimator.cpp
  src/estimation_manager/estimators/agl_estimator.cpp
  )

add_dependencies(PairsUavManagers_EstimationManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_EstimationManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# PairsUavManagers_TransformManager
add_library(PairsUavManagers_TransformManager
  src/transform_manager/transform_manager.cpp
  )

add_dependencies(PairsUavManagers_TransformManager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavManagers_TransformManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING AND PAIRS_ENABLE_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/pairs_uav_managers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
