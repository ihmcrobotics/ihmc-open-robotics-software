package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.util.PeriodicThreadScheduler;

public interface QuadrupedControllerManager extends RobotController
{
   RobotMotionStatusHolder getMotionStatusHolder();
   void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator);
}
