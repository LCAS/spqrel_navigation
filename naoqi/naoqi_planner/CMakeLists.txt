qi_create_lib(naoqi_planner_library
  SHARED
  naoqi_planner.cpp naoqi_planner.h
  dynamic_map.cpp dynamic_map.h
  )
qi_use_lib(naoqi_planner_library
  srrg_path_map_library
  naoqi_sensor_utils_library
  system_utils
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  )
qi_stage_lib(naoqi_planner_library)

qi_create_lib(naoqi_planner_gui_library
  SHARED
  naoqi_planner_gui.cpp naoqi_planner_gui.h
  )
qi_use_lib(naoqi_planner_gui_library
  srrg_path_map_library
  naoqi_sensor_utils_library
  yaml_parser_library
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
