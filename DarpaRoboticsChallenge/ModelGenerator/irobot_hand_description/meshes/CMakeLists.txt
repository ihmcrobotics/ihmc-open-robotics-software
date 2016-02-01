set (files 
finger_base.stl
finger_distal.stl
finger_proximal.stl
palm.stl
)
install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/irobot_hand_description/meshes/)
