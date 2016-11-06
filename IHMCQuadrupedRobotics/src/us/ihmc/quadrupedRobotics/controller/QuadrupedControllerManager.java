package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

public interface QuadrupedControllerManager extends RobotController
{
   RobotMotionStatusHolder getMotionStatusHolder();
}
