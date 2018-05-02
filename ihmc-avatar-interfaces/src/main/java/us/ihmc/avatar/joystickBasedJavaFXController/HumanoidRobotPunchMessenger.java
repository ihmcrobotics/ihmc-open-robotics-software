package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HumanoidRobotPunchMessenger
{
   void sendArmHomeConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide... robotSides);

   void sendArmStraightConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide robotSide);
}
