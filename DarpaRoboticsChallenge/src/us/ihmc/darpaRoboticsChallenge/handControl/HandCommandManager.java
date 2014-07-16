package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class HandCommandManager
{
	private static final String TCP_PORT = "4270";
	   
	private JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	private KryoObjectServer server = new KryoObjectServer(Integer.parseInt(TCP_PORT), new DRCNetClassList());
	
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
	
	public void spawnHandControllerThreadManager(Class<? extends Object> clazz)
	{
		spawner.spawn(clazz, new String[]{"--port", TCP_PORT});
	}
	
	public void sendHandCommand(Object packet)
	{
		server.consumeObject(packet);
	}
	
	public String getTcpPort()
	{
		return TCP_PORT;
	}
}
