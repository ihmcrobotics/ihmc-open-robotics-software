package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.FilteredPacketSendingForwarder;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class UiConnectionModule implements PacketConsumer<RobotConfigurationData>
{
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   
   private final FilteredPacketSendingForwarder packetForwarderFromUiModuleToUserInterface;
   private final FilteredPacketSendingForwarder packetForwarderFromUserInterfaceToUiModule;
   
   private final Class<?>[] packetsAllowedToGotoUserInterface = PacketsForwardedToTheUi.PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE;
   private final HashMap<Class<?>, Long> packetsAllowedToGotoUserInterfaceWithIntervals = PacketsForwardedToTheUi.PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS;
   
   private final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();
   
   private final PacketCommunicator uiPacketServer = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, NETCLASSLIST);
   private final PacketCommunicator uiModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, NETCLASSLIST);
   
   public UiConnectionModule()
   {
      //Forward all packets from the User Interface to the network processor
      packetForwarderFromUserInterfaceToUiModule = new FilteredPacketSendingForwarder(uiPacketServer, uiModuleCommunicator, timestampProvider);
      
      //Selective send packets to the User Interface
      packetForwarderFromUiModuleToUserInterface = new FilteredPacketSendingForwarder(uiModuleCommunicator, uiPacketServer, timestampProvider);
      uiModuleCommunicator.attachListener(RobotConfigurationData.class, this);
      
      setPacketForwardingFilters();
      connect();
   }
   
   private void setPacketForwardingFilters()
   {
      packetForwarderFromUserInterfaceToUiModule.enableUnfilteredForwarding();
      
      packetForwarderFromUiModuleToUserInterface.enableInclusiveForwarding(packetsAllowedToGotoUserInterface);
      
      for (Entry<Class<?>, Long> entry : packetsAllowedToGotoUserInterfaceWithIntervals.entrySet())
      {
         packetForwarderFromUiModuleToUserInterface.enableInclusiveForwardingWithMinimumRobotTimeIntervals(entry.getKey(), entry.getValue());
      }
   }
   
   
   public void connect()
   {
      try
      {
         uiPacketServer.connect();
         uiModuleCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public boolean isConnected()
   {
      return uiPacketServer.isConnected();
   }
   
   public void close()
   {
      uiPacketServer.disconnect();
      uiModuleCommunicator.disconnect();
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      timestampProvider.set(packet.getMonotonicTime());
   }
}
