set (files 
  base_link.STL
  hokuyo_link.STL
  head.dae
  head_camera.dae
)
install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/multisense_sl_description/meshes/)
