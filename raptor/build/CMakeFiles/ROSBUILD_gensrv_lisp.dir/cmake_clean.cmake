FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/raptor/msg"
  "../src/raptor/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/obstacle_histogram.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_obstacle_histogram.lisp"
  "../srv_gen/lisp/polar_histogram.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_polar_histogram.lisp"
  "../srv_gen/lisp/twist_srv.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_twist_srv.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
