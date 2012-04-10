FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rovio_shared/msg"
  "../src/rovio_shared/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/man_drv.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_man_drv.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
