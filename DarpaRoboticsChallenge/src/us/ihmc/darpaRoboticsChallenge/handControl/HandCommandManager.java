package us.ihmc.darpaRoboticsChallenge.handControl;

import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;
import us.ihmc.utilities.robotSide.RobotSide;

public abstract class HandCommandManager
{
   private final boolean DEBUG = true;
   private final String SERVER_ADDRESS = "localhost";

   protected final PacketCommunicatorMock handManagerPacketCommunicator;

   protected JavaProcessSpawner spawner = new JavaProcessSpawner(true);

   protected PacketCommunicatorMock packetCommunicator;

   public HandCommandManager(Class<? extends Object> clazz, RobotSide robotSide)
   {
      // decided to decouple the comms startup process for the hands
      // HandCommandManager should only spawn a HndControlThreadManager when debugging
      if (DEBUG)
         spawner.spawn(clazz, new String[] { "-r", robotSide.getLowerCaseName() });

      NetworkPorts managerPort = robotSide == RobotSide.LEFT ? NetworkPorts.LEFT_HAND_MANAGER_PORT : NetworkPorts.RIGHT_HAND_MANAGER_PORT;
      handManagerPacketCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(managerPort, new IHMCCommunicationKryoNetClassList());

      NetworkPorts port = robotSide.equals(RobotSide.LEFT) ? NetworkPorts.LEFT_HAND_PORT : NetworkPorts.RIGHT_HAND_PORT;
      packetCommunicator = PacketCommunicatorMock.createTCPPacketCommunicatorClient(SERVER_ADDRESS, port, new IHMCCommunicationKryoNetClassList());

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
