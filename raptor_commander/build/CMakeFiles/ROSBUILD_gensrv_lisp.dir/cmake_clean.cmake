FILE(REMOVE_RECURSE
  "../src/raptor_commander/msg"
  "../src/raptor_commander/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/getAdvicePD.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getAdvicePD.lisp"
  "../srv_gen/lisp/getAdviceFD.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getAdviceFD.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
