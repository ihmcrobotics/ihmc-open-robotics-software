package us.ihmc.robotiq.control;

import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointAnglePacket;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorControllerStateHandler;
import us.ihmc.utilities.net.ObjectConsumer;

public class RobotiqHandCommandManager extends HandCommandManager
{
	public RobotiqHandCommandManager(final DRCNetworkProcessorControllerStateHandler controllerStateHandler)
	{
		super(RobotiqControlThreadManager.class);
		
		server.attachListener(HandJointAnglePacket.class, new ObjectConsumer<HandJointAnglePacket>()
		{
			public void consumeObject(HandJointAnglePacket object)
			{
				controllerStateHandler.sendHandJointAnglePacket(object);
			}
		});
	}
}
