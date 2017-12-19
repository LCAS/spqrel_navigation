#set(CMAKE_VERBOSE_MAKEFILE ON)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

set (CMAKE_BUILD_TYPE Release)

include_directories(../src)

#add_subdirectory(srrg_system_utils)
#add_subdirectory(yaml_parser)

qi_create_lib(srrg_system_utils_yaml
  SHARED
  ../src/srrg_system_utils/system_utils.cpp 
  ../src/yaml_parser/simple_yaml_parser.cpp 
)

qi_use_lib(srrg_system_utils_yaml
  EIGEN3
)

qi_stage_lib(srrg_system_utils_yaml)


#add_subdirectory(srrg_path_map)


qi_create_lib(srrg_path_map_library SHARED
  ../src/srrg_path_map/path_map.cpp 
  ../src/srrg_path_map/path_map_utils.cpp 
  ../src/srrg_path_map/base_path_search.cpp 
  ../src/srrg_path_map/distance_map_path_search.cpp 
  ../src/srrg_path_map/dijkstra_path_search.cpp
)

qi_use_lib(srrg_path_map_library
  OPENCV2_CORE
  EIGEN3
)

qi_stage_lib(srrg_path_map_library)

qi_create_bin(distance_map_test 
    ../src/srrg_path_map/distance_map_test.cpp
)

qi_use_lib(distance_map_test
  srrg_path_map_library
  srrg_system_utils_yaml
  OPENCV2_CORE
  OPENCV2_HIGHGUI
)

qi_create_bin(dijkstra_test 
    ../src/srrg_path_map/dijkstra_test.cpp
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
  ../src/srrg_localizer2d/localization_filter.cpp
)

qi_use_lib(srrg_localizer2d_library
  srrg_path_map_library
  OPENCV2_CORE
  OPENCV2_IMGPROC
)

qi_stage_lib(srrg_localizer2d_library)


#add_subdirectory(../src/srrg_planner2d)
qi_create_lib(srrg_planner2d_library
  ../src/srrg_planner2d/planner.cpp
  ../src/srrg_planner2d/dynamic_map.cpp
  ../src/srrg_planner2d/motion_controller.cpp
)

qi_use_lib(srrg_planner2d_library
  srrg_system_utils_yaml
  srrg_path_map_library
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  BOOST
)

qi_stage_lib(srrg_planner2d_library)


add_subdirectory(naoqi_sensor_utils)
add_subdirectory(naoqi_localizer)
add_subdirectory(naoqi_planner)
add_subdirectory(naoqi_navigation_gui)

