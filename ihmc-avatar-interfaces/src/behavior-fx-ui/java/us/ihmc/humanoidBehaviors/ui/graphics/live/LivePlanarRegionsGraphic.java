package us.ihmc.humanoidBehaviors.ui.graphics.live;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class LivePlanarRegionsGraphic extends AnimationTimer
{
   private final PlanarRegionsGraphic planarRegionsGraphic = new PlanarRegionsGraphic();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   public LivePlanarRegionsGraphic(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.REA_MODULE, this::acceptPlanarRegions);
   }

   private void acceptPlanarRegions(PlanarRegionsListMessage incomingData)
   {
      executorService.submit(() -> { // important not to execute this in either ROS2 or JavaFX threads
         planarRegionsGraphic.generateMeshes(PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData));
      });
   }

   @Override
   public void handle(long now)
   {
      planarRegionsGraphic.update();
   }

   public Node getRoot()
   {
      return planarRegionsGraphic.getRoot();
   }
}
