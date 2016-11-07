package us.ihmc.avatar.handControl;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class HandControlThread implements Runnable
{
   protected PacketCommunicator packetCommunicator;
	
   public HandControlThread(RobotSide robotSide)
   {
      NetworkPorts port = robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_PORT : NetworkPorts.RIGHT_HAND_PORT;
      String host = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, new IHMCCommunicationKryoNetClassList(), true);
   }
   
   public abstract void connect();
}
