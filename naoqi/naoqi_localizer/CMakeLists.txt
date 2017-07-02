qi_create_lib(naoqi_localizer_library  
  SHARED
  naoqi_localizer.cpp naoqi_localizer.h
  )

qi_use_lib(naoqi_localizer_library 
  srrg_localizer2d_library
  naoqi_sensor_utils_library
  system_utils
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  )
qi_stage_lib(naoqi_localizer_library)


qi_create_bin(pepper_localizer pepper_localizer.cpp)
qi_use_lib(pepper_localizer ALCOMMON ALPROXIES naoqi_localizer_library)
