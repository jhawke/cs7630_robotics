FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/raptor/msg"
  "../src/raptor/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/raptor/srv/__init__.py"
  "../src/raptor/srv/_obstacle_histogram.py"
  "../src/raptor/srv/_polar_histogram.py"
  "../src/raptor/srv/_distance_adv_srv.py"
  "../src/raptor/srv/_light_level_srv.py"
  "../src/raptor/srv/_twist_srv.py"
  "../src/raptor/srv/_state_set_srv.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
