FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/raptor/msg"
  "../src/raptor/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/raptor/obstacle_histogram.h"
  "../srv_gen/cpp/include/raptor/position_request_srv.h"
  "../srv_gen/cpp/include/raptor/polar_histogram.h"
  "../srv_gen/cpp/include/raptor/twist_srv.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
