FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rovio_shared/msg"
  "../src/rovio_shared/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/rovio_shared/rovio_position.h"
  "../srv_gen/cpp/include/rovio_shared/wav_play.h"
  "../srv_gen/cpp/include/rovio_shared/head_ctrl.h"
  "../srv_gen/cpp/include/rovio_shared/twist_srv.h"
  "../srv_gen/cpp/include/rovio_shared/man_drv_srv.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
