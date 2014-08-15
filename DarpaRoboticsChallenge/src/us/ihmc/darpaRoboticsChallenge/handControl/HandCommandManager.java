package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointAnglePacket;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorControllerStateHandler;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class HandCommandManager
{
	private static final String TCP_PORT = "4270";
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	protected KryoObjectServer server = new KryoObjectServer(Integer.parseInt(TCP_PORT), new DRCNetClassList());
	
	public HandCommandManager(final DRCNetworkProcessorControllerStateHandler controllerStateHandler, Class<? extends Object> clazz)
	{
		server.attachListener(HandJointAnglePacket.class, new ObjectConsumer<HandJointAnglePacket>()
      {
         @Override
         public void consumeObject(HandJointAnglePacket object)
         {
            controllerStateHandler.sendHandJointAnglePacket(object);
         }
      });
		
		spawnHandControllerThreadManager(clazz);
		
		try
		{
			server.connect();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
	
	private void spawnHandControllerThreadManager(Class<? extends Object> clazz)
	{
		spawner.spawn(clazz, new String[]{"--port", TCP_PORT});
	}
   
	public void sendHandCommand(Object packet)
	{
	   server.consumeObject(packet);
	}
}
