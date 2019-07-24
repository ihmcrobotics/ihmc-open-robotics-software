package us.ihmc.humanoidBehaviors.ui.graphics.live;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class LivePlanarRegionsGraphic extends PlanarRegionsGraphic
{
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   public LivePlanarRegionsGraphic(Ros2Node ros2Node)
   {
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA, this::acceptPlanarRegions);

      animationTimer.start();
   }

   private void acceptPlanarRegions(PlanarRegionsListMessage incomingData)
   {
      executorService.submit(() -> { // important not to execute this in either ROS2 or JavaFX threads
         super.generateMeshes(PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData));
      });
   }

   private void handle(long now)
   {
      super.update();
   }
}
