cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
# Give a name to the project.
project(spqrel_navigation)

#set(CMAKE_VERBOSE_MAKEFILE ON)

if(qibuild_DIR)
  # This include enable you to use qibuild framework
  find_package(qibuild)

  message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
  message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

  include_directories(./src ./naoqi)

  #add_subdirectory(srrg_system_utils)
  #add_subdirectory(yaml_parser)

  qi_create_lib(srrg_system_utils_yaml
    SHARED
    ./src/srrg_system_utils/system_utils.cpp 
    ./src/yaml_parser/simple_yaml_parser.cpp 
  )

  qi_use_lib(srrg_system_utils_yaml
    EIGEN3
  )

  qi_stage_lib(srrg_system_utils_yaml)


  #add_subdirectory(srrg_path_map)


  qi_create_lib(srrg_path_map_library SHARED
    ./src/srrg_path_map/path_map.cpp 
    ./src/srrg_path_map/path_map_utils.cpp 
    ./src/srrg_path_map/base_path_search.cpp 
    ./src/srrg_path_map/distance_map_path_search.cpp 
    ./src/srrg_path_map/dijkstra_path_search.cpp
  )

  qi_use_lib(srrg_path_map_library
    OPENCV2_CORE
    EIGEN3
  )

  qi_stage_lib(srrg_path_map_library)

  qi_create_bin(distance_map_test 
      ./src/srrg_path_map/distance_map_test.cpp
  )

  qi_use_lib(distance_map_test
    srrg_path_map_library
    srrg_system_utils_yaml
    OPENCV2_CORE
    OPENCV2_HIGHGUI
  )

  qi_create_bin(dijkstra_test 
      ./src/srrg_path_map/dijkstra_test.cpp
  )

  qi_use_lib(dijkstra_test
    srrg_path_map_library
    srrg_system_utils_yaml
    OPENCV2_CORE
    OPENCV2_HIGHGUI
    OPENCV2_IMGPROC
  )

  #add_subdirectory(../src/srrg_localizer2d)


  qi_create_lib(srrg_localizer2d_library
    ./src/srrg_localizer2d/localization_filter.cpp
  )

  qi_use_lib(srrg_localizer2d_library
    srrg_path_map_library
    OPENCV2_CORE
    OPENCV2_IMGPROC
  )

  qi_stage_lib(srrg_localizer2d_library)


  add_subdirectory(naoqi/naoqi_sensor_utils)
  add_subdirectory(naoqi/naoqi_localizer)
  add_subdirectory(naoqi/naoqi_planner)


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
    message_generation
  )

  set(CMAKE_CXX_FLAGS "-std=c++11")

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
    ./src/srrg_system_utils/system_utils.cpp
    ./src/srrg_path_map/base_path_search.cpp
    ./src/srrg_path_map/distance_map_path_search.cpp
    ./src/srrg_path_map/dijkstra_path_search.cpp
    ./src/srrg_path_map/path_map.cpp
    ./src/srrg_localizer2d/localization_filter.cpp
    ./src/srrg_system_utils/system_utils.cpp
    ./src/srrg_path_map/base_path_search.cpp
    ./src/srrg_path_map/distance_map_path_search.cpp
    ./src/srrg_path_map/dijkstra_path_search.cpp
    ./src/srrg_path_map/path_map.cpp
    ./ros/srrg_ros_wrappers/ros_utils.cpp
  )

  add_dependencies(srrg_ros 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(srrg_ros
    ${catkin_LIBRARIES}
  )



  ###############
  add_executable(srrg_localizer2d_node 
    ros/srrg_localizer2d_ros/src/ros_localizer.cpp
    ros/srrg_localizer2d_ros/src/srrg_localizer2d_node.cpp
  )


  ## Add cmake target dependencies of the executable
  ## same as for the library above
  add_dependencies(srrg_localizer2d_node 
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
  install(TARGETS srrg_localizer2d_node srrg_ros # ros_planner_node
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
