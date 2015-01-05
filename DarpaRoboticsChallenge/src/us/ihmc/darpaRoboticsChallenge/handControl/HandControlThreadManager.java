package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packets.PacketDestination;

public abstract class HandControlThreadManager
{
   protected KryoPacketServer packetCommunicator;
	
	public HandControlThreadManager(int tcpPort)
	{
	   packetCommunicator = new KryoPacketServer(tcpPort, new IHMCCommunicationKryoNetClassList(), PacketDestination.HAND_MANAGER.ordinal(), "HandCommandManagerServerCommunicator");
	}
	
   public abstract void start();
	
   public abstract void connect();
}
