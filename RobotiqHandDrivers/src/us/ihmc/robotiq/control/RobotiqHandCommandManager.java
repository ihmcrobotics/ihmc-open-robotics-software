package us.ihmc.robotiq.control;

import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.net.ObjectConsumer;
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
      networkManager.getControllerCommandHandler().attachListener(FingerStatePacket.class, new ObjectConsumer<FingerStatePacket>()
      {
         public void consumeObject(FingerStatePacket object)
         {
            sendHandCommand(object);
         }
      });
      
      networkManager.getControllerCommandHandler().attachListener(ManualHandControlPacket.class, new ObjectConsumer<ManualHandControlPacket>()
      {
         public void consumeObject(ManualHandControlPacket object)
         {
            sendHandCommand(object);
         }
      });
   }

   protected void setupOutboundPacketListeners()
   {
      server.attachListener(HandJointAnglePacket.class, new ObjectConsumer<HandJointAnglePacket>()
      {
         public void consumeObject(HandJointAnglePacket object)
         {
            networkManager.getControllerStateHandler().sendSerializableObject(object);
         }
      });
   }
}
