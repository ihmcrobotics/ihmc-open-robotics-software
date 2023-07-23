package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import perception_msgs.msg.dds.ManuallyPlacedSceneNodeMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class SceneGraphAPI
{
   public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("scene_graph");
   public static final ROS2Topic<DetectableSceneNodesMessage> DETECTABLE_SCENE_NODES = BASE_TOPIC.withTypeName(DetectableSceneNodesMessage.class).withOutput();
   public static final ROS2Topic<ManuallyPlacedSceneNodeMessage> PLACED_SCENE_NODE = BASE_TOPIC.withTypeName(ManuallyPlacedSceneNodeMessage.class).withOutput();
}
