package us.ihmc.perception.scene;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.ros2.ROS2Topic;

public class ROS2SceneObjectAPI
{
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_FRAME = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_pull_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_PANEL = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_pull_door_panel");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_FRAME = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_push_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_PANEL = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class)
                                                                                                            .withSuffix("detected_push_door_panel");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_BOX = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_box");
}
