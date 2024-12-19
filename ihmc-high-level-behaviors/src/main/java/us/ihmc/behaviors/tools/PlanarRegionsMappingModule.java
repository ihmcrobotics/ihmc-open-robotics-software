package us.ihmc.behaviors.tools;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

/**
 * Builds and provides a map by subscribing to REA.
 */
public class PlanarRegionsMappingModule
{
   private final ROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;

   private volatile PlanarRegionsList slamMap = new PlanarRegionsList();
   private PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();

   private Notification slamUpdated = new Notification();

   private long i = 0; // to slow down trying to SLAM for now
   private long sequenceId = 0; // to detect slam updated
   private static final int SLAM_EVERY = 1;

   public PlanarRegionsMappingModule()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build(PerceptionAPI.MAPPING_MODULE_NODE_NAME);

      planarRegionPublisher = ros2Node.createPublisher(PerceptionAPI.REALSENSE_SLAM_MODULE.withOutput().withTypeName(PlanarRegionsListMessage.class));
      ROS2Topic realsenseTopic = PerceptionAPI.REA.withOutput().withSuffix("realsense");
      ros2Node.createSubscription2(realsenseTopic.withType(PlanarRegionsListMessage.class), this::process);
      ros2Node.createSubscription2(PerceptionAPI.LIDAR_REA_REGIONS.withType(PlanarRegionsListMessage.class), this::process);
   }

   private void process(PlanarRegionsListMessage visibleRegionsMessage)
   {
      boolean slamUpdatedTemp = false;

      if (slamMap.isEmpty())
      {
         slamMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(visibleRegionsMessage);
         slamUpdatedTemp = true;
      }
      else
      {
         if (i++ % SLAM_EVERY == 0)
         {
            try
            {
               PlanarRegionsList visibleRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(visibleRegionsMessage);
               slamMap = PlanarRegionSLAM.slam(slamMap, visibleRegions, planarRegionSLAMParameters, (ConcaveHullMergerListener) null).getMergedMap();
               slamUpdatedTemp = true;
            }
            catch (Exception e)
            {
               // do nothing but need to fix these crashes
            }
         }
      }

      PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(slamMap);
      if (slamUpdatedTemp)
      {
         sequenceId++;
      }
      message.setSequenceId(sequenceId);
      planarRegionPublisher.publish(message);

      if (slamUpdatedTemp)
      {
         slamUpdated.set();
      }
   }

   public PlanarRegionsList getLatestMap()
   {
      return slamMap;
   }

   public Notification getSlamUpdated()
   {
      return slamUpdated;
   }
}
