FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/raptor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/raptor/obstacle_histogram.h"
  "../srv_gen/cpp/include/raptor/polar_histogram.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
