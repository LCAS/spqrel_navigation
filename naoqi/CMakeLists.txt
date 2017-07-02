cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
# Give a name to the project.
project(pepper_navigation)

#set(CMAKE_VERBOSE_MAKEFILE ON)

# This include enable you to use qibuild framework
find_package(qibuild)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(../src)

#add_subdirectory(../src/srrg_system_utils)
#add_subdirectory(../src/yaml_parser)

qi_create_lib(system_utils
  SHARED
  ../src/yaml_parser/simple_yaml_parser.cpp #simple_yaml_parser.h
  ../src/srrg_system_utils/system_utils.cpp #system_utils.h
  )

qi_use_lib(system_utils  
  EIGEN3
  )

qi_stage_lib(system_utils)


#add_subdirectory(../src/srrg_path_map)


qi_create_lib(srrg_path_map_library SHARED
  ../src/srrg_path_map/path_map.cpp #path_map.h
  ../src/srrg_path_map/path_map_utils.cpp #path_map_utils.h
  ../src/srrg_path_map/base_path_search.cpp #base_path_search.h
  ../src/srrg_path_map/distance_map_path_search.cpp #distance_map_path_search.h
  ../src/srrg_path_map/dijkstra_path_search.cpp #dijkstra_path_search.h
  )

qi_use_lib(srrg_path_map_library
  OPENCV2_CORE
  EIGEN3
  )
qi_stage_lib(srrg_path_map_library)

#qi_create_bin(distance_map_test distance_map_test.cpp)
#qi_use_lib(distance_map_test
#  srrg_path_map_library
#  srrg_system_utils_library
#  OPENCV2_CORE
#  OPENCV2_HIGHGUI
#  )

#qi_create_bin(dijkstra_test dijkstra_test.cpp)
#qi_use_lib(dijkstra_test
#  srrg_path_map_library
#  srrg_system_utils_library
#  OPENCV2_CORE
#  OPENCV2_HIGHGUI
#  OPENCV2_IMGPROC
#  )

#add_subdirectory(../src/srrg_localizer2d)


qi_create_lib(srrg_localizer2d_library
  ../src/srrg_localizer2d/localization_filter.cpp #localization_filter.h
  )

qi_use_lib(srrg_localizer2d_library
  srrg_path_map_library
  OPENCV2_CORE
  OPENCV2_IMGPROC
  )

qi_stage_lib(srrg_localizer2d_library)



add_subdirectory(naoqi_sensor_utils)
add_subdirectory(naoqi_localizer)
add_subdirectory(naoqi_planner)

