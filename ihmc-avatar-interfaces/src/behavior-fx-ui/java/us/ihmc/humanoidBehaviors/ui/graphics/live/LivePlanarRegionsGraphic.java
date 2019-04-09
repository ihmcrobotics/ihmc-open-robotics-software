package us.ihmc.humanoidBehaviors.ui.graphics.live;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.commons.collections4.ListUtils;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class LivePlanarRegionsGraphic extends PlanarRegionsGraphic
{
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private volatile PlanarRegionsList lastREARegions = new PlanarRegionsList();
   private volatile PlanarRegionsList lastSupportRegions = new PlanarRegionsList();

   public LivePlanarRegionsGraphic(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      new ROS2Callback<>(ros2Node,
                         PlanarRegionsListMessage.class,
                         robotModel.getSimpleRobotName(),
                         LIDARBasedREAModule.ROS2_ID,
                         this::acceptPlanarRegions);
      new ROS2Callback<>(ros2Node,
                         PlanarRegionsListMessage.class,
                         null,
                         BipedalSupportPlanarRegionPublisher.ROS2_ID.getModuleTopicQualifier(),
                         ROS2TopicQualifier.INPUT,
                         this::acceptSupportRegions);

      animationTimer.start();
   }

   private void acceptPlanarRegions(PlanarRegionsListMessage incomingData)
   {
      executorService.submit(() -> { // important not to execute this in either ROS2 or JavaFX threads
         lastREARegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData);

         generateMeshes();
      });
   }

   private void acceptSupportRegions(PlanarRegionsListMessage incomingData)
   {
      executorService.submit(() -> { // important not to execute this in either ROS2 or JavaFX threads
         lastSupportRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData);

         generateMeshes();
      });
   }

   private synchronized void generateMeshes() // synchronized because it's called from two separate threads (above)
   {
      super.generateMeshes(new PlanarRegionsList(ListUtils.union(lastREARegions.getPlanarRegionsAsList(), lastSupportRegions.getPlanarRegionsAsList())));
   }

   private void handle(long now)
   {
      super.update();
   }
}
