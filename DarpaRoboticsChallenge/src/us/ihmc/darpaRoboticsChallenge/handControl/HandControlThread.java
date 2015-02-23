package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packets.PacketDestination;

public abstract class HandControlThread
{
   private final int TCP_PORT = 4270; // should match port in HandCommandManager
   protected KryoPacketCommunicator packetCommunicator = new KryoPacketServer(TCP_PORT, new IHMCCommunicationKryoNetClassList(),
         PacketDestination.HAND_MANAGER.ordinal(), "HandThreadManagerServerCommunicator");
	
   public abstract void start();
	
   public abstract void connect();
}
