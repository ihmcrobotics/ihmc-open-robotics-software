package us.ihmc.avatar.ros2;

import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;
import java.util.function.Function;

public class ROS2ControllerHelper extends ROS2Helper implements ROS2PublishSubscribeAPI, ROS2ControllerPublishSubscribeAPI
{
   protected final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private final DRCRobotModel robotModel;

   public ROS2ControllerHelper(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      super(ros2Node);
      this.robotModel = robotModel;
      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(robotModel.getSimpleRobotName(), ros2PublisherMap);
   }

   @Override
   public <T> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      subscribeViaCallback(topicFunction.apply(robotModel.getSimpleRobotName()), callback);
   }

   @Override
   public void publishToController(Object message)
   {
      ros2ControllerPublisherMap.publish(message);
   }

   @Override
   public <T> void publish(Function<String, ROS2Topic<T>> topic, T message)
   {
      publish(topic.apply(robotModel.getSimpleRobotName()), message);
   }

   @Override
   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaCallback(ControllerAPIDefinition.getTopic(messageClass, robotModel.getSimpleRobotName()), callback);
   }

   @Override
   public Notification subscribeToWalkingCompletedViaNotification()
   {
      Notification notification = new Notification();
      subscribeToControllerViaCallback(WalkingStatusMessage.class, walkingStatusMessage -> {
         if (walkingStatusMessage.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
         {
            notification.set();
         }
      });
      return notification;
   }
}
