FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rovio_shared/msg"
  "../src/rovio_shared/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rovio_shared/man_drv.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
