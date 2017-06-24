qi_create_lib(srrg_path_map_library SHARED
  path_map.cpp path_map.h
  path_map_utils.cpp path_map_utils.h
  base_path_search.cpp base_path_search.h
  distance_map_path_search.cpp distance_map_path_search.h
  dijkstra_path_search.cpp dijkstra_path_search.h
  )

qi_use_lib(srrg_path_map_library
  OPENCV2_CORE
  EIGEN3
  )
qi_stage_lib(srrg_path_map_library)

qi_create_bin(distance_map_test distance_map_test.cpp)
qi_use_lib(distance_map_test
  srrg_path_map_library
  srrg_system_utils_library
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  )

qi_create_bin(dijkstra_test dijkstra_test.cpp)
qi_use_lib(dijkstra_test
  srrg_path_map_library
  srrg_system_utils_library
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_IMGPROC
  )
