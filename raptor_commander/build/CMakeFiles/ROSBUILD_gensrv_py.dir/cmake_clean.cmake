FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/raptor_commander/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/raptor_commander/srv/__init__.py"
  "../src/raptor_commander/srv/_getAdviceFLEE.py"
  "../src/raptor_commander/srv/_getAdviceDET.py"
  "../src/raptor_commander/srv/_getAdviceFD.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
