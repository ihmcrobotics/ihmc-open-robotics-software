
p_x = root0x2EYoboticsBipedalRobot0x2EProcessedSensors0x2Ep_x_vicon; 
p_y = root0x2EYoboticsBipedalRobot0x2EProcessedSensors0x2Ep_y_vicon; 
p_z = root0x2EYoboticsBipedalRobot0x2EProcessedSensors0x2Ep_z_vicon;

viconCovariance = (cov(throwAwayRepeated(p_x)) + cov(throwAwayRepeated(p_y)) + cov(throwAwayRepeated(p_z))) / 3;
accelerationCovariance = (cov(pdd_x_world) + cov(pdd_y_world) + cov(pdd_z_world)) / 3;
orientationCovariance = (cov(p_yaw) + cov(p_pitch) + cov(p_roll)) / 3;