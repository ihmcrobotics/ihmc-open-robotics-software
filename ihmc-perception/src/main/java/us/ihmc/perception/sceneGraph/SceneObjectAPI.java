package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class SceneObjectAPI
{
   public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("scene_object");
   public static final ROS2Topic<DetectableSceneNodesMessage> DETECTABLE_SCENE_OBJECTS = BASE_TOPIC.withTypeName(DetectableSceneNodesMessage.class);
}
