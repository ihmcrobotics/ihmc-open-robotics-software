package us.ihmc.robotiq.control;

import us.ihmc.communication.NetworkProcessorControllerStateHandler;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.utilities.net.ObjectConsumer;

public class RobotiqHandCommandManager extends HandCommandManager
{
	public RobotiqHandCommandManager(final NetworkProcessorControllerStateHandler controllerStateHandler)
	{
		super(RobotiqControlThreadManager.class);
		
		server.attachListener(HandJointAnglePacket.class, new ObjectConsumer<HandJointAnglePacket>()
		{
			public void consumeObject(HandJointAnglePacket object)
			{
				controllerStateHandler.sendSerializableObject(object);
			}
		});
	}
}
