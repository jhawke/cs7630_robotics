FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rovio_shared/msg"
  "../src/rovio_shared/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/rovio_shared/srv/__init__.py"
  "../src/rovio_shared/srv/_rovio_position.py"
  "../src/rovio_shared/srv/_wav_play.py"
  "../src/rovio_shared/srv/_head_ctrl.py"
  "../src/rovio_shared/srv/_twist_srv.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
