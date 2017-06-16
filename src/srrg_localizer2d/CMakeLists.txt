qi_create_lib(srrg_localizer2d_library
  localization_filter.cpp localization_filter.h
  )

qi_use_lib(srrg_localizer2d_library
  srrg_path_map_library
  OPENCV2_CORE
  OPENCV2_IMGPROC
  )
qi_stage_lib(srrg_localizer2d_library)
