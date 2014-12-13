package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public abstract class HandCommandManager
{
	private static final String TCP_PORT = "4270";
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	protected KryoObjectServer server = new KryoObjectServer(Integer.parseInt(TCP_PORT), new IHMCCommunicationKryoNetClassList());
	
	public HandCommandManager(Class<? extends Object> clazz)
	{
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
	
	protected abstract void setupInboundPacketListeners();
	protected abstract void setupOutboundPacketListeners();
}
