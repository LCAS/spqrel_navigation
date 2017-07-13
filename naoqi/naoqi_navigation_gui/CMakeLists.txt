qi_create_lib(naoqi_navigation_gui_library
  SHARED
  naoqi_navigation_gui.cpp
)

qi_use_lib(naoqi_navigation_gui_library
  srrg_system_utils_yaml
  srrg_path_map_library
  naoqi_sensor_utils_library
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_IMGPROC
)

qi_stage_lib(naoqi_navigation_gui_library)

qi_create_bin(pepper_navigation_gui pepper_navigation_gui.cpp)
qi_use_lib(pepper_navigation_gui ALCOMMON ALPROXIES naoqi_navigation_gui_library)

