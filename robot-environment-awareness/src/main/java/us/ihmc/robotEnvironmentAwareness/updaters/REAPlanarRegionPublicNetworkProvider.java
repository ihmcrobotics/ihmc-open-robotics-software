package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

public class REAPlanarRegionPublicNetworkProvider
{
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;

   private final AtomicBoolean hasReceivedClearRequest = new AtomicBoolean(false);
   private final RegionFeaturesProvider regionFeaturesProvider;

   public REAPlanarRegionPublicNetworkProvider(RegionFeaturesProvider regionFeaturesProvider, Ros2Node ros2Node,
                                               MessageTopicNameGenerator publisherTopicNameGenerator, MessageTopicNameGenerator subscriberTopicNameGenerator)
   {
      this.regionFeaturesProvider = regionFeaturesProvider;
      publisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, publisherTopicNameGenerator);
      ROS2Tools.createCallbackSubscription(ros2Node, RequestPlanarRegionsListMessage.class, subscriberTopicNameGenerator, this::handlePacket);
   }

   public void update(boolean planarRegionsHaveBeenUpdated)
   {
      processRequests();

      if (regionFeaturesProvider.getPlanarRegionsList() == null)
         return;

      if (regionFeaturesProvider.getPlanarRegionsList().isEmpty())
         return;

      publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList()));
   }

   private void processRequests()
   {
      while (!requestsToProcess.isEmpty())
      {
         RequestPlanarRegionsListMessage request = requestsToProcess.poll();
         PlanarRegionsRequestType requestType = PlanarRegionsRequestType.fromByte(request.getPlanarRegionsRequestType());
         if (requestType == PlanarRegionsRequestType.CLEAR)
            hasReceivedClearRequest.set(true);
      }
   }

   public boolean pollClearRequest()
   {
      return hasReceivedClearRequest.getAndSet(false);
   }

   private final ConcurrentLinkedQueue<RequestPlanarRegionsListMessage> requestsToProcess = new ConcurrentLinkedQueue<>();

   private void handlePacket(Subscriber<RequestPlanarRegionsListMessage> subscriber)
   {
      requestsToProcess.offer(subscriber.takeNextData());
   }
}
