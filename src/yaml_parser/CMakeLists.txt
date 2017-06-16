qi_create_lib(yaml_parser_library  
  SHARED
  simple_yaml_parser.cpp simple_yaml_parser.h
  )
qi_use_lib(yaml_parser_library  
  EIGEN3
  )
qi_stage_lib(yaml_parser_library)
