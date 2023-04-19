package us.ihmc.perception.scene;

import perception_msgs.msg.dds.DetectableSceneObjectsMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class SceneObjectAPI
{
   public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("scene_object");
   public static final ROS2Topic<DetectableSceneObjectsMessage> DETECTABLE_SCENE_OBJECTS = BASE_TOPIC.withTypeName(DetectableSceneObjectsMessage.class);
}
