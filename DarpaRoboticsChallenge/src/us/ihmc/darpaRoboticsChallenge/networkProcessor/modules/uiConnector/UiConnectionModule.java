package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.FilteredPacketSendingForwarder;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.util.NetworkConfigParameters;

public class UiConnectionModule implements PacketConsumer<RobotConfigurationData>
{
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   
   private final FilteredPacketSendingForwarder packetForwarderFromUiModuleToUserInterface;
   private final FilteredPacketSendingForwarder packetForwarderFromUserInterfaceToUiModule;
   
   private final Class[] packetsAllowedToGotoUserInterface = PacketsForwardedToTheUi.PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE;
   private final HashMap<Class, Long> packetsAllowedToGotoUserInterfaceWithIntervals = PacketsForwardedToTheUi.PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS;
   
   private final int NP_TO_UI_TCP_PORT = NetworkConfigParameters.NETWORK_PROCESSOR_TO_UI_RAW_PROTOCOL_TCP_PORT;
   private final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();
   private final KryoPacketServer uiKryoPacketServer = new KryoPacketServer(NP_TO_UI_TCP_PORT, NETCLASSLIST, PacketDestination.UI.ordinal(),"UIServerCommunicator"); //connection to the UI
   private final KryoLocalPacketCommunicator uiModuleCommunicator = new KryoLocalPacketCommunicator(NETCLASSLIST, PacketDestination.UI.ordinal(),"UIModuleCommunicator"); // connect to network processor
   
   public UiConnectionModule()
   {
      //Forward all packets from the User Interface to the network processor
      packetForwarderFromUserInterfaceToUiModule = new FilteredPacketSendingForwarder(uiKryoPacketServer, uiModuleCommunicator, timestampProvider);
      
      //Selective send packets to the User Interface
      packetForwarderFromUiModuleToUserInterface = new FilteredPacketSendingForwarder(uiModuleCommunicator, uiKryoPacketServer, timestampProvider);
      uiModuleCommunicator.attachListener(RobotConfigurationData.class, this);
      
      setPacketForwardingFilters();
      connect();
   }
   
   private void setPacketForwardingFilters()
   {
      packetForwarderFromUserInterfaceToUiModule.enableUnfilteredForwarding();
      
      packetForwarderFromUiModuleToUserInterface.enableInclusiveForwarding(packetsAllowedToGotoUserInterface);
      
      for (Entry<Class, Long> entry : packetsAllowedToGotoUserInterfaceWithIntervals.entrySet())
      {
         packetForwarderFromUiModuleToUserInterface.enableInclusiveForwardingWithMinimumRobotTimeIntervals(entry.getKey(), entry.getValue());
      }
   }
   
   public KryoPacketCommunicator getPacketCommunicator()
   {
      return uiModuleCommunicator;
   }
   
   public void connect()
   {
      try
      {
         uiKryoPacketServer.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public boolean isConnected()
   {
      return uiKryoPacketServer.isConnected();
   }
   
   public void close()
   {
      uiKryoPacketServer.close();
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      timestampProvider.set(packet.getTimestamp());
   }
}
