FILE(REMOVE_RECURSE
  "../src/raptor_commander/msg"
  "../src/raptor_commander/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/abs_pos_req.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_abs_pos_req.lisp"
  "../msg_gen/lisp/darkness_region.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_darkness_region.lisp"
  "../msg_gen/lisp/rel_pos_req.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_rel_pos_req.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
