package us.ihmc.robotiq.control;

import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;

public class RobotiqHandCommandManager extends HandCommandManager
{
   private final AbstractNetworkProcessorNetworkingManager networkManager;
   
	public RobotiqHandCommandManager(final AbstractNetworkProcessorNetworkingManager networkManager)
	{
		super(RobotiqControlThreadManager.class);
		
		this.networkManager = networkManager;
		
		if(networkManager != null)
		{
			setupOutboundPacketListeners();
			setupInboundPacketListeners();
		}
	}

   protected void setupInboundPacketListeners()
   {
      networkManager.getControllerCommandHandler().attachListener(FingerStatePacket.class, new PacketConsumer<FingerStatePacket>()
      {
         public void receivedPacket(FingerStatePacket object)
         {
            sendHandCommand(object);
         }
      });
      
      networkManager.getControllerCommandHandler().attachListener(ManualHandControlPacket.class, new PacketConsumer<ManualHandControlPacket>()
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
            networkManager.getControllerStateHandler().sendPacket(object);
         }
      });
   }
}
