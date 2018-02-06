package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;

public class REAPlanarRegionPublicNetworkProvider
{
   private final Set<PacketDestination> listenersForContinuousUpdate = new HashSet<>();
   private final Set<PacketDestination> listenersForSingleUpdate = new HashSet<>();
   private final AtomicBoolean hasReceivedClearRequest = new AtomicBoolean(false);
   private final PacketCommunicator publicPacketCommunicator;
   private final RegionFeaturesProvider regionFeaturesProvider;

   public REAPlanarRegionPublicNetworkProvider(RegionFeaturesProvider regionFeaturesProvider, PacketCommunicator publicPacketCommunicator)
   {
      this.regionFeaturesProvider = regionFeaturesProvider;
      this.publicPacketCommunicator = publicPacketCommunicator;
      publicPacketCommunicator.attachListener(RequestPlanarRegionsListMessage.class, this::handlePacket);
   }
   
   public void update(boolean planarRegionsHaveBeenUpdated)
   {
      processRequests();

      boolean hasAtLeastOneListener = !listenersForContinuousUpdate.isEmpty() || !listenersForSingleUpdate.isEmpty();

      if (!hasAtLeastOneListener || regionFeaturesProvider.getPlanarRegionsList() == null)
         return;

      // By doing so, this module does not flood the network with useless data.
      if (planarRegionsHaveBeenUpdated)
      {
         for (PacketDestination packetDestination : listenersForContinuousUpdate)
         {
            PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());
            planarRegionsListMessage.setDestination(packetDestination);
            publicPacketCommunicator.send(planarRegionsListMessage);
         }
      }

      for (PacketDestination packetDestination : listenersForSingleUpdate)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());
         planarRegionsListMessage.setDestination(packetDestination);
         publicPacketCommunicator.send(planarRegionsListMessage);
      }

      listenersForSingleUpdate.clear();
   }

   private void processRequests()
   {
      while (!requestsToProcess.isEmpty())
      {
         RequestPlanarRegionsListMessage request = requestsToProcess.poll();
         PacketDestination source = PacketDestination.fromOrdinal(request.getSource());
         RequestType requestType = request.getRequestType();
         switch (requestType)
         {
         case CONTINUOUS_UPDATE:
            listenersForContinuousUpdate.add(source);
            break;
         case SINGLE_UPDATE:
            listenersForSingleUpdate.add(source);
            break;
         case STOP_UPDATE:
            listenersForContinuousUpdate.remove(source);
            break;
         case CLEAR:
            hasReceivedClearRequest.set(true);
            break;
         default:
            break;
         }
      }
   }

   public boolean pollClearRequest()
   {
      return hasReceivedClearRequest.getAndSet(false);
   }

   private final ConcurrentLinkedQueue<RequestPlanarRegionsListMessage> requestsToProcess = new ConcurrentLinkedQueue<>();

   private void handlePacket(RequestPlanarRegionsListMessage packet)
   {
      if (packet != null)
         requestsToProcess.offer(packet);
   }
}
