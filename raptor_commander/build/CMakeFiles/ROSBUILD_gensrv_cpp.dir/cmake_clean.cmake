FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/raptor_commander/msg"
  "../src/raptor_commander/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/raptor_commander/getAdvicePD.h"
  "../srv_gen/cpp/include/raptor_commander/getAdviceFD.h"
  "../srv_gen/cpp/include/raptor_commander/switchState.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
