package us.ihmc.humanoidBehaviors.ui.graphics.live;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

public class LivePlanarRegionsGraphic extends AnimationTimer
{
   private final PlanarRegionsGraphic planarRegionsGraphic = new PlanarRegionsGraphic();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   public LivePlanarRegionsGraphic(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class,
                                           ROS2Tools.getTopicNameGenerator(null,
                                                                           ROS2Tools.REA_MODULE,
                                                                           ROS2Tools.ROS2TopicQualifier.OUTPUT),
                                           this::rosCallback);
   }

   private void rosCallback(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage incomingData = subscriber.takeNextData(); // may be 1 or 2 ticks behind, is this okay?
      if (incomingData != null)
      {
         executorService.submit(() -> { // important not to execute this in either ROS2 or JavaFX threads
            planarRegionsGraphic.generateMeshes(PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData));
         });
      }
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
