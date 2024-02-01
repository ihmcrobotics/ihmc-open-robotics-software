package us.ihmc.behaviors.tools.perception;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Combines and publishes planar regions from suppliers.
 */
public class CompositePlanarRegionService
{
   private Supplier<PlanarRegionsList>[] planarRegionSuppliers;
   private final List<IHMCROS2Publisher<PlanarRegionsListMessage>> planarRegionPublishers = new ArrayList<>();
   private final IHMCROS2Publisher<PlanarRegionsListMessage> combinedPlanarRegionPublisher;
   private final PausablePeriodicThread thread;

   private PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();
   private long sequenceId = 0;

   public CompositePlanarRegionService(ROS2NodeInterface ros2Node, List<String> topicNames, double period, Supplier<PlanarRegionsList>... planarRegionSuppliers)
   {
      this.planarRegionSuppliers = planarRegionSuppliers;

      for (int i = 0; i < planarRegionSuppliers.length; i++)
      {
         planarRegionPublishers.add(ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, topicNames.get(i)));
      }

      combinedPlanarRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, PerceptionAPI.MAP_REGIONS);
      thread = new PausablePeriodicThread(getClass().getSimpleName(), period, this::process);
   }

   public void start()
   {
      thread.start();
   }

   public void stop()
   {
      thread.stop();
   }

   private void process()
   {
      if (planarRegionSuppliers.length <= 0)
      {
         return;
      }

      PlanarRegionsListMessage message;

      PlanarRegionsList planarRegionsListToPublish = planarRegionSuppliers[0].get();
      message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsListToPublish);
      planarRegionPublishers.get(0).publish(message);

      for (int i = 1; i < planarRegionSuppliers.length; i++)
      {
         PlanarRegionsList planarRegions = planarRegionSuppliers[i].get();
         message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegions);
         planarRegionPublishers.get(i).publish(message);

         planarRegionsListToPublish = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(planarRegionsListToPublish,
                                                                                                         planarRegions,
                                                                                                         planarRegionSLAMParameters,
                                                                                                         null);
      }

      message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsListToPublish);
      message.setSequenceId(++sequenceId);
      combinedPlanarRegionPublisher.publish(message);
   }
}
