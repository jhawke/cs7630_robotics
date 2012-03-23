FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/raptor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/raptor/srv/__init__.py"
  "../src/raptor/srv/_obstacle_histogram.py"
  "../src/raptor/srv/_polar_histogram.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
