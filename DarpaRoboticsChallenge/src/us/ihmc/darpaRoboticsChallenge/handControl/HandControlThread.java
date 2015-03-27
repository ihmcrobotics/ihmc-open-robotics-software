package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.robotSide.RobotSide;

public abstract class HandControlThread implements Runnable
{
   protected KryoPacketCommunicator packetCommunicator;
	
   public HandControlThread(RobotSide robotSide)
   {
      packetCommunicator = new KryoPacketServer(robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_PORT.getPort() : NetworkPorts.RIGHT_HAND_PORT.getPort(),
                                                new IHMCCommunicationKryoNetClassList(),
                                                robotSide.equals(RobotSide.LEFT) ? PacketDestination.LEFT_HAND_MANAGER.ordinal() : PacketDestination.RIGHT_HAND_MANAGER.ordinal(),
                                                robotSide.getCamelCaseNameForStartOfExpression() + "HandControlThreadServerCommunicator");
   }
   
   public abstract void connect();
}
