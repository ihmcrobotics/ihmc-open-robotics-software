package us.ihmc.robotiq.control;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommandManager extends HandCommandManager
{
	public RobotiqHandCommandManager(RobotSide robotSide)
	{
	   super(RobotiqControlThread.class, robotSide);
	   
	   setupOutboundPacketListeners();
	   setupInboundPacketListeners();
	}

   protected void setupInboundPacketListeners()
   {
      handManagerPacketCommunicator.attachListener(FingerStatePacket.class, new PacketConsumer<FingerStatePacket>()
      {
         public void receivedPacket(FingerStatePacket object)
         {
            sendHandCommand(object);
         }
      });
      
      handManagerPacketCommunicator.attachListener(ManualHandControlPacket.class, new PacketConsumer<ManualHandControlPacket>()
      {
         public void receivedPacket(ManualHandControlPacket object)
         {
            sendHandCommand(object);
         }
      });
   }

   protected void setupOutboundPacketListeners()
   {
      packetCommunicator.attachListener(HandJointAnglePacket.class, new PacketConsumer<HandJointAnglePacket>()
      {
         public void receivedPacket(HandJointAnglePacket object)
         {
            handManagerPacketCommunicator.send(object);
         }
      });
   }
}
