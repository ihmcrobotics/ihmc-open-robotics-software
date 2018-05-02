package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;

public interface HumanoidRobotLowLevelMessenger
{
   void sendFreezeRequest(PacketCommunicator packetCommunicator);

   void sendStandRequest(PacketCommunicator packetCommunicator);
}
