package us.ihmc.avatar.ros2;

import controller_msgs.RobotConfigurationData;
import controller_msgs.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.util.function.Consumer;
import java.util.function.Function;

public class ROS2ControllerHelper extends ROS2Helper
{
   private final DRCRobotModel robotModel;
   private final String simpleRobotName;
   protected final ROS2ControllerPublisherMap ros2ControllerPublisherMap;

   public ROS2ControllerHelper(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      this(ros2Node, robotModel.getSimpleRobotName(), robotModel);
   }

   public ROS2ControllerHelper(ROS2Node ros2Node, String simpleRobotName)
   {
      this(ros2Node, simpleRobotName, null);
   }

   private ROS2ControllerHelper(ROS2Node ros2Node, String simpleRobotName, DRCRobotModel robotModel)
   {
      super(ros2Node);
      this.robotModel = robotModel;
      this.simpleRobotName = simpleRobotName;
      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(simpleRobotName, ros2PublisherMap);
   }

   public <T extends ROS2Message<T>> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      subscribeViaCallback(topicFunction.apply(simpleRobotName), callback);
   }

   public <T extends ROS2Message<T>> void subscribeViaVolatileCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      subscribeViaVolatileCallback(topicFunction.apply(simpleRobotName), callback);
   }

   public <T extends ROS2Message<T>> void publishToController(T message)
   {
      ros2ControllerPublisherMap.publish(message);
   }

   public <T extends ROS2Message<T>> void publish(Function<String, ROS2Topic<T>> topic, T message)
   {
      publish(topic.apply(simpleRobotName), message);
   }

   public <T extends ROS2Message<T>> ROS2Input<T> subscribeToController(Class<T> messageClass)
   {
      return subscribe(HumanoidControllerAPI.getTopic(messageClass, simpleRobotName));
   }

   public <T extends ROS2Message<T>> ROS2Input<T> subscribeToControllerLowFrequency(Class<T> messageClass)
   {
      return subscribe(HumanoidControllerAPI.getLowFrequencyTopic(messageClass, simpleRobotName));
   }

   public ROS2Input<RobotConfigurationData> subscribeToRobotConfigurationData()
   {
      return subscribe(StateEstimatorAPI.getRobotConfigurationDataTopic(getRobotName()));
   }

   public <T extends ROS2Message<T>> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaCallback(HumanoidControllerAPI.getTopic(messageClass, simpleRobotName), callback);
   }

   public <T extends ROS2Message<T>> void subscribeToControllerViaVolatileCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaVolatileCallback(HumanoidControllerAPI.getTopic(messageClass, simpleRobotName), callback);
   }

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

   public String getRobotName()
   {
      return simpleRobotName;
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
