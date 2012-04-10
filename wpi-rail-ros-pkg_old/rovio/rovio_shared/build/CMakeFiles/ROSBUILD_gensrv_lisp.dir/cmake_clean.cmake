FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rovio_shared/msg"
  "../src/rovio_shared/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/rovio_position.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_rovio_position.lisp"
  "../srv_gen/lisp/wav_play.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_wav_play.lisp"
  "../srv_gen/lisp/head_ctrl.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_head_ctrl.lisp"
  "../srv_gen/lisp/twist_srv.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_twist_srv.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
