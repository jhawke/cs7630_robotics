FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/raptor_commander/msg"
  "../src/raptor_commander/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/raptor_commander/msg/__init__.py"
  "../src/raptor_commander/msg/_darkness_region.py"
  "../src/raptor_commander/msg/_blob_colour.py"
  "../src/raptor_commander/msg/_abs_pos_req.py"
  "../src/raptor_commander/msg/_rel_pos_req.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
