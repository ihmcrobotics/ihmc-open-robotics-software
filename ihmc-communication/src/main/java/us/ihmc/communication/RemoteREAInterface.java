package us.ihmc.communication;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2PublisherBasics;

import java.util.function.Consumer;

public class RemoteREAInterface
{
   private final ROS2PublisherBasics<REAStateRequestMessage> reaStateRequestPublisher;
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsListInput;

   private final Stopwatch stopwatch = new Stopwatch();

   public RemoteREAInterface(ROS2NodeInterface ros2Node)
   {
      reaStateRequestPublisher = ros2Node.createPublisher(PerceptionAPI.REA.withInput().withTypeName(REAStateRequestMessage.class));
      planarRegionsListInput = new ROS2Input<>(ros2Node, PerceptionAPI.LIDAR_REA_REGIONS);

      planarRegionsListInput.addCallback(planarRegionsListMessage -> stopwatch.start());
   }

   public void addPlanarRegionsListCallback(Consumer<PlanarRegionsList> planarRegionsListConsumer)
   {
      planarRegionsListInput.addCallback(planarRegionsListMessage ->
      {
         planarRegionsListConsumer.accept(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      });
   }

   public PlanarRegionsList getLatestPlanarRegionsList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListInput.getLatest());
   }

   public PlanarRegionsListMessage getLatestPlanarRegionsListMessage()
   {
      return planarRegionsListInput.getLatest();
   }

   public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   /**
    * NaN is no planar regions yet received.
    * @return time since planar regions last updated
    */
   public double timeSinceLastUpdate()
   {
      return stopwatch.lapElapsed();
   }

   // TODO Remove this from this class? It might not belong here
   public boolean getPlanarRegionsListExpired(double expirationDuration)
   {
      return Double.isNaN(timeSinceLastUpdate()) || timeSinceLastUpdate() > expirationDuration;
   }
}
