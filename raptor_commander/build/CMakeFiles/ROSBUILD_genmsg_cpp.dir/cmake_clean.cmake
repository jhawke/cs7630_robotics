FILE(REMOVE_RECURSE
  "../src/raptor_commander/msg"
  "../src/raptor_commander/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/raptor_commander/abs_pos_req.h"
  "../msg_gen/cpp/include/raptor_commander/darkness_region.h"
  "../msg_gen/cpp/include/raptor_commander/blob_colour.h"
  "../msg_gen/cpp/include/raptor_commander/rel_pos_req.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
