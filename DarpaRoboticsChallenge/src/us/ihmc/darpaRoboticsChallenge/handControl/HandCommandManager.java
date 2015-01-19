package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketClient;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;
//TODO: rename and create interface for sending hand command
public abstract class HandCommandManager //implements HandCommandInterface
{
	private static final String TCP_PORT = "4270";
	private static final String SERVER_ADDRESS = "localhost";
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	
	
	protected KryoPacketClient packetCommunicator = new KryoPacketClient(SERVER_ADDRESS, NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
         new IHMCCommunicationKryoNetClassList(),PacketDestination.CONTROLLER.ordinal(),PacketDestination.NETWORK_PROCESSOR.ordinal(),"AtlasROSAPINetworkProcessor");
	
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
}
