package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketClient;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packets.PacketDestination;

public abstract class HandControlThreadManager
{
//   protected KryoPacketServer packetCommunicator;
   protected KryoPacketClient packetCommunicator;
	
	public HandControlThreadManager(int tcpPort)
	{
	   packetCommunicator = new KryoPacketClient("localhost", 4270, //NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
	         new IHMCCommunicationKryoNetClassList(),PacketDestination.HAND_MANAGER.ordinal(),PacketDestination.HAND_MANAGER.ordinal(),"AtlasROSAPINetworkProcessor");
	}
	
   public abstract void start();
	
   public abstract void connect();
}
