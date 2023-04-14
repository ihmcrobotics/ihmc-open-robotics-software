package us.ihmc.perception.scene;

import perception_msgs.msg.dds.DetectableSceneObjectMessage;
import us.ihmc.ros2.ROS2Topic;

public class ROS2SceneObjectAPI
{
   public static final ROS2Topic<DetectableSceneObjectMessage> PULL_DOOR_FRAME
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("pull_door_frame");
   public static final ROS2Topic<DetectableSceneObjectMessage> PULL_DOOR_PANEL
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("pull_door_panel");
   public static final ROS2Topic<DetectableSceneObjectMessage> PULL_DOOR_LEVER_HANDLE
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("pull_door_lever_handle");
   public static final ROS2Topic<DetectableSceneObjectMessage> PUSH_DOOR_FRAME
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("push_door_frame");
   public static final ROS2Topic<DetectableSceneObjectMessage> PUSH_DOOR_PANEL
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("push_door_panel");
   public static final ROS2Topic<DetectableSceneObjectMessage> PUSH_DOOR_LEVER_HANDLE
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("push_door_lever_handle");
   public static final ROS2Topic<DetectableSceneObjectMessage> BOX
         = SceneObjectAPI.BASE_TOPIC.withType(DetectableSceneObjectMessage.class).withSuffix("box");
}
