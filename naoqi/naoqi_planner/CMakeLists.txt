qi_create_lib(naoqi_planner_library
  SHARED
  naoqi_planner.cpp 
)

qi_use_lib(naoqi_planner_library
  srrg_system_utils_yaml
  srrg_path_map_library
  srrg_planner2d_library
  naoqi_sensor_utils_library
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  OPENCV2_HIGHGUI
)

qi_stage_lib(naoqi_planner_library)

qi_create_lib(naoqi_planner_gui_library
  SHARED
  naoqi_planner_gui.cpp
)

qi_use_lib(naoqi_planner_gui_library
  srrg_system_utils_yaml
  srrg_path_map_library
  naoqi_sensor_utils_library
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  OPENCV2_HIGHGUI
)

qi_stage_lib(naoqi_planner_gui_library)


qi_create_bin(pepper_planner pepper_planner.cpp)
qi_use_lib(pepper_planner ALCOMMON ALPROXIES naoqi_planner_library)


qi_create_bin(pepper_planner_gui pepper_planner_gui.cpp)
qi_use_lib(pepper_planner_gui ALCOMMON ALPROXIES naoqi_planner_gui_library)


