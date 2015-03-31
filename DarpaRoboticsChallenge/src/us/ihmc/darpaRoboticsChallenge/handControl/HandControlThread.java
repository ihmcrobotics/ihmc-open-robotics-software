package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.robotSide.RobotSide;

public abstract class HandControlThread implements Runnable
{
   protected PacketCommunicator packetCommunicator;
	
   public HandControlThread(RobotSide robotSide)
   {
      NetworkPorts port = robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_PORT : NetworkPorts.RIGHT_HAND_PORT;
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(port, new IHMCCommunicationKryoNetClassList());
   }
   
   public abstract void connect();
}
