package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketClientEndPointCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;
import us.ihmc.utilities.robotSide.RobotSide;

public abstract class HandCommandManager
{
   private final boolean DEBUG = false;
	private final String SERVER_ADDRESS = "localhost";
	   
	protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);
	
	protected KryoPacketClientEndPointCommunicator packetCommunicator;
	
	public HandCommandManager(Class<? extends Object> clazz, RobotSide robotSide)
	{
	   // decided to decouple the comms startup process for the hands
	   // HandCommandManager should only spawn a HndControlThreadManager when debugging
	   if(DEBUG)
	      spawner.spawn(clazz, new String[]{"-r", robotSide.getLowerCaseName()});
		
		packetCommunicator = new KryoPacketClientEndPointCommunicator(SERVER_ADDRESS,
		                                                              robotSide.equals(RobotSide.LEFT) ? NetworkConfigParameters.LEFT_HAND_PORT : NetworkConfigParameters.RIGHT_HAND_PORT,
		                                                              new IHMCCommunicationKryoNetClassList(),
		                                                              robotSide.equals(RobotSide.LEFT) ? PacketDestination.LEFT_HAND_MANAGER.ordinal() : PacketDestination.RIGHT_HAND_MANAGER.ordinal(),
		                                                              robotSide.getCamelCaseNameForStartOfExpression() + "HandCommandManagerClient");
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
	
	public void sendHandCommand(Packet packet)
	{
	   packetCommunicator.send(packet);
	}
	
	protected abstract void setupInboundPacketListeners();
	protected abstract void setupOutboundPacketListeners();
	public abstract PacketCommunicator getCommunicator();
}
