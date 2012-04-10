FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/raptor_commander/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/raptor_commander/getAdviceFLEE.h"
  "../srv_gen/cpp/include/raptor_commander/getAdviceDET.h"
  "../srv_gen/cpp/include/raptor_commander/getAdviceFD.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
