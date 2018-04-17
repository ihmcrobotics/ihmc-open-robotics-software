package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import java.io.IOException;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.FilteredPacketSendingForwarder;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;

public class ControllerFilteredConnectionModule
{
   private final FilteredPacketSendingForwarder packetForwarderFromFilterModuleToController;
   private final FilteredPacketSendingForwarder packetForwarderFromControllerToFilterModule;

   private final Class<?>[] packetsAllowedToGotoController = PacketsAllowedToController.PACKETS_ALLOWED_TO_CONTROLLER;

   private final PacketCommunicator controllerPacketClient;
   private final PacketCommunicator controllerModuleCommunicator;

   public ControllerFilteredConnectionModule(NetClassList netClassList)
   {
      PrintTools.info(this, "Connecting to controller using TCP on " + NetworkParameters.getHost(NetworkParameterKeys.robotController));
      controllerPacketClient = PacketCommunicator.createTCPPacketCommunicatorClient(NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                                                    NetworkPorts.CONTROLLER_PORT, netClassList);
      controllerModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_FILTER_MODULE_PORT, netClassList);
      //Forward all packets from the Controller to the network processor 
      packetForwarderFromControllerToFilterModule = new FilteredPacketSendingForwarder(controllerPacketClient, controllerModuleCommunicator, null);

      //Selective send packets to the Controller
      packetForwarderFromFilterModuleToController = new FilteredPacketSendingForwarder(controllerModuleCommunicator, controllerPacketClient, null);

      setPacketForwardingFilters();
      connect();
   }

   private void setPacketForwardingFilters()
   {
      packetForwarderFromControllerToFilterModule.enableUnfilteredForwarding();
      packetForwarderFromFilterModuleToController.enableInclusiveForwarding(packetsAllowedToGotoController);
   }

   public void connect()
   {
      try
      {
         controllerPacketClient.connect();
         controllerModuleCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public boolean isConnected()
   {
      return controllerPacketClient.isConnected();
   }

   public void close()
   {
      controllerPacketClient.disconnect();
      controllerModuleCommunicator.disconnect();
   }
}
