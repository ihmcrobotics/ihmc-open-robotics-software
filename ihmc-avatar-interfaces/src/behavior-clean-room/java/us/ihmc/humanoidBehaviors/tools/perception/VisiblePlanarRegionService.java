package us.ihmc.humanoidBehaviors.tools.perception;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2TopicName;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.function.Supplier;

/**
 * Assembles and publishes currently visible planar regions on the REA output topic.
 */
public class VisiblePlanarRegionService
{
   private Supplier<PlanarRegionsList>[] planarRegionSuppliers;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private final PausablePeriodicThread thread;

   private PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();

   public VisiblePlanarRegionService(Ros2NodeInterface ros2Node, Supplier<PlanarRegionsList>... planarRegionSuppliers)
   {
      this.planarRegionSuppliers = planarRegionSuppliers;
      planarRegionPublisher = new IHMCROS2Publisher<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REA.suffix(ROS2TopicName.OUTPUT)); // TODO add name "visible"
      thread = new PausablePeriodicThread(getClass().getSimpleName(), 0.5, this::process);
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

      PlanarRegionsList planarRegionsListToPublish = planarRegionSuppliers[0].get();

      for (int i = 1; i < planarRegionSuppliers.length; i++)
      {
         planarRegionsListToPublish = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(planarRegionsListToPublish,
                                                                                                         planarRegionSuppliers[i].get(),
                                                                                                         planarRegionSLAMParameters,
                                                                                                         null);
      }

      PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsListToPublish);
      planarRegionPublisher.publish(message);
   }
}
