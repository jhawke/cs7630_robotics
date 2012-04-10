FILE(REMOVE_RECURSE
  "../src/raptor_commander/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/getAdviceFLEE.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getAdviceFLEE.lisp"
  "../srv_gen/lisp/getAdviceDET.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getAdviceDET.lisp"
  "../srv_gen/lisp/getAdviceFD.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getAdviceFD.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
