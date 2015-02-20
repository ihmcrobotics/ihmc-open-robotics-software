package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketClientEndPointCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;
//TODO: rename and create interface for sending hand command
public abstract class HandCommandManager //implements HandCommandInterface
{
   private final boolean DEBUG = true;
	private final String SERVER_ADDRESS = "localhost";
	private final int TCP_PORT = 4270; // should match port in HandControlThreadManager
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	
	protected KryoPacketClientEndPointCommunicator packetCommunicator;
	
	public HandCommandManager(Class<? extends Object> clazz)
	{
	   // decided to decouple the comms startup process for the hands
	   // HandCommandManager should only spawn a HndControlThreadManager when debugging
	   if(DEBUG)
	      spawnHandControllerThreadManager(clazz);
		
		packetCommunicator = new KryoPacketClientEndPointCommunicator(SERVER_ADDRESS, TCP_PORT,
	         new IHMCCommunicationKryoNetClassList(), PacketDestination.HAND_MANAGER.ordinal(), "HandCommandManagerClient");
		packetCommunicator.setReconnectAutomatically(true);
		
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
		spawner.spawn(clazz);
	}
   
	public void sendHandCommand(Packet packet)
	{
	   packetCommunicator.send(packet);
	}
	
	protected abstract void setupInboundPacketListeners();
	protected abstract void setupOutboundPacketListeners();
	public abstract PacketCommunicator getCommunicator();
}
