package us.ihmc.communication;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.function.Consumer;

public class RemoteEnvironmentMapInterface
{
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsListInput;

   private final Stopwatch stopwatch = new Stopwatch();

   public RemoteEnvironmentMapInterface(Ros2NodeInterface ros2Node)
   {
      planarRegionsListInput = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_MAP_TOPIC_NAME);

      planarRegionsListInput.addCallback(planarRegionsListMessage -> stopwatch.start());
   }

   public void addPlanarRegionsListCallback(Consumer<PlanarRegionsList> planarRegionsListConsumer)
   {
      planarRegionsListInput.addCallback(planarRegionsListMessage ->
                                               planarRegionsListConsumer.accept(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage)));
   }

   public PlanarRegionsList getLatestPlanarRegionsList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListInput.getLatest());
   }

   public PlanarRegionsListMessage getLatestPlanarRegionsListMessage()
   {
      return planarRegionsListInput.getLatest();
   }

   /**
    * NaN is no planar regions yet received.
    * @return time since planar regions last updated
    */
   public double timeSinceLastUpdate()
   {
      return stopwatch.lapElapsed();
   }

   public boolean getPlanarRegionsListExpired(double expirationDuration)
   {
      return Double.isNaN(timeSinceLastUpdate()) || timeSinceLastUpdate() > expirationDuration;
   }
}
