qi_create_lib(naoqi_sensor_utils_library SHARED
  naoqi_sensor_utils.cpp naoqi_sensor_utils.h
  )
qi_use_lib(naoqi_sensor_utils_library
  EIGEN3
  ALCOMMON 
  ALPROXIES
  OPENCV2_CORE
  )
qi_stage_lib(naoqi_sensor_utils_library)
