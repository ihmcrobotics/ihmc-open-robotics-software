package us.ihmc.perception.objects;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.perception.scene.SceneObjectAPI;
import us.ihmc.ros2.ROS2Topic;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class BasicSceneObjects
{
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_BOX = SceneObjectAPI.BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_box");
}
