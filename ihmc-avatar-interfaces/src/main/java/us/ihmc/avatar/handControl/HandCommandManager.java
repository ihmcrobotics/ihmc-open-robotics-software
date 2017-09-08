package us.ihmc.avatar.handControl;

import java.io.IOException;

import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public abstract class HandCommandManager
{
   private final String SERVER_ADDRESS = "localhost";

   protected final PacketCommunicator handManagerPacketCommunicator;
   protected final PacketCommunicator packetCommunicator;

   protected JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);


   public HandCommandManager(Class<? extends Object> clazz, RobotSide robotSide)
   {
      String networkParamProperty = System.getProperty("us.ihmc.networkParameterFile", NetworkParameters.defaultParameterFile);
      
      spawner.spawn(clazz, new String[] {"-Dus.ihmc.networkParameterFile=" + networkParamProperty}, new String[] { "-r", robotSide.getLowerCaseName() });

      NetworkPorts managerPort = robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_MANAGER_PORT : NetworkPorts.RIGHT_HAND_MANAGER_PORT;
      handManagerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(managerPort, new IHMCCommunicationKryoNetClassList());

      NetworkPorts port = robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_PORT : NetworkPorts.RIGHT_HAND_PORT;
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(port, new IHMCCommunicationKryoNetClassList());

      try
      {
         packetCommunicator.connect();
         handManagerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void sendHandCommand(Packet<?> packet)
   {
      packetCommunicator.send(packet);
   }

   protected abstract void setupInboundPacketListeners();

   protected abstract void setupOutboundPacketListeners();

}
