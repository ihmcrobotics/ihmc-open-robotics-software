package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;
//TODO: rename and create interface for sending hand command
public abstract class HandCommandManager //implements HandCommandInterface
{
	private static final String TCP_PORT = "4270";
	private static final String SERVER_ADDRESS = "localhost";
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	
	protected KryoPacketServer packetCommunicator = new KryoPacketServer(4270, new IHMCCommunicationKryoNetClassList(), PacketDestination.HAND_MANAGER.ordinal(), "HandCommandManagerServerCommunicator");
	
	public HandCommandManager(Class<? extends Object> clazz)
	{
		spawnHandControllerThreadManager(clazz);
		try
		{
		   packetCommunicator.connect();
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
   
	public void sendHandCommand(Packet packet)
	{
	   packetCommunicator.send(packet);
	}
	
	protected abstract void setupInboundPacketListeners();
	protected abstract void setupOutboundPacketListeners();
	public abstract PacketCommunicator getCommunicator();
}
