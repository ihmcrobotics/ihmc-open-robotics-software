package us.ihmc.aware.controller;

import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public interface QuadrupedControllerManager extends RobotController
{
   RobotMotionStatusHolder getMotionStatusHolder();
}
