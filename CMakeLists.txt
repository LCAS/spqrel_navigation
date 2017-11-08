cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
# Give a name to the project.
project(spqrel_navigation)

#set(CMAKE_VERBOSE_MAKEFILE ON)

if(qibuild_DIR)

  find_package(qibuild)

  add_subdirectory(naoqi)


else(qibuild_DIR)

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    cv_bridge
    tf
    move_base_msgs
    message_generation
    dynamic_reconfigure
  )

  set(CMAKE_CXX_FLAGS "-std=c++11")
  set(CMAKE_BUILD_TYPE Release)

  ## Generate messages in the 'msg' folder
  add_message_files(
    FILES
    LocalizerRanges.msg
  )



  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    sensor_msgs
  )


  catkin_package(
#    INCLUDE_DIRS include
    LIBRARIES srrg_ros
    CATKIN_DEPENDS geometry_msgs nav_msgs nav_msgs sensor_msgs cv_bridge 
#    DEPENDS Eigen3
  )

  FIND_PACKAGE(OpenCV REQUIRED)
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

  message("OpenCV version: ${OPENCV_VERSION_MAJOR}.${OPENCV_VERSION_MINOR}.${OPENCV_VERSION_PATCH}")


  # Find Eigen3
  FIND_PACKAGE(Eigen3 REQUIRED)
  INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ./src/
    ./src/srrg_localizer2d
    ./ros/
    ./ros/srrg_ros_wrappers
  )


  add_library(srrg_ros
    ./src/yaml_parser/simple_yaml_parser.cpp
    ./src/srrg_system_utils/system_utils.cpp
    ./src/srrg_path_map/base_path_search.cpp
    ./src/srrg_path_map/distance_map_path_search.cpp
    ./src/srrg_path_map/dijkstra_path_search.cpp
    ./src/srrg_path_map/path_map.cpp
    ./src/srrg_path_map/path_map_utils.cpp
    ./src/srrg_localizer2d/localization_filter.cpp
    ./src/srrg_system_utils/system_utils.cpp
    ./src/srrg_path_map/base_path_search.cpp
    ./src/srrg_path_map/distance_map_path_search.cpp
    ./src/srrg_path_map/dijkstra_path_search.cpp
    ./src/srrg_path_map/path_map.cpp
    ./ros/srrg_ros_wrappers/ros_utils.cpp
    ./src/srrg_planner2d/motion_controller.cpp
    ./src/srrg_planner2d/dynamic_map.cpp
    ./src/srrg_planner2d/planner.cpp
  )

  add_dependencies(srrg_ros 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(srrg_ros
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )



  ###############
  add_executable(srrg_localizer2d_node 
    ros/srrg_localizer2d_ros/src/ros_localizer.cpp
    ros/srrg_localizer2d_ros/src/srrg_localizer2d_node.cpp
  )

  add_executable(spqrel_planner_node
    ros/spqrel_planner/src/ros_planner.cpp
    ros/spqrel_planner/src/ros_planner_node.cpp
  )

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  add_dependencies(srrg_localizer2d_node 
    srrg_ros
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    spqrel_navigation_generate_messages_cpp
  )

  add_dependencies(spqrel_planner_node 
    srrg_ros
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    spqrel_navigation_generate_messages_cpp
  )

  ## Specify libraries to link a library or executable target against
  target_link_libraries(srrg_localizer2d_node
    srrg_ros
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS} 
  )

  target_link_libraries(spqrel_planner_node
    srrg_ros
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS} 
  )
  
  ###############
  
  # DISABLED until it actually compiles:
  # add_executable(ros_planner_node 
  #   ./ros/ros_planner/src/ros_planner.cpp
  #   ./ros/ros_planner/src/ros_planner_node.cpp
  # )

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  #  add_dependencies(srrg_localizer2d_node 
  #    srrg_ros
  #    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  #    ${catkin_EXPORTED_TARGETS}
  #  )

  # ## Specify libraries to link a library or executable target against
  # target_link_libraries(srrg_localizer2d_node
  #   srrg_ros
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBS} 
  # )


  #############
  ## Install ##
  #############


  ## Mark executables and/or libraries for installation
  install(TARGETS srrg_localizer2d_node spqrel_planner_node srrg_ros 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  ## Mark cpp header files for installation
  # install(DIRECTORY include/${PROJECT_NAME}/
  #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #   FILES_MATCHING PATTERN "*.h"
  #   PATTERN ".svn" EXCLUDE
  # )

endif(qibuild_DIR)
