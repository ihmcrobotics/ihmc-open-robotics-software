package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.ros2.ROS2Topic;

public class SceneGraphAPI
{
   public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("scene_graph");
   public static final ROS2IOTopicPair<DetectableSceneNodesMessage> DETECTABLE_SCENE_NODES
                 = new ROS2IOTopicPair<>(BASE_TOPIC.withTypeName(DetectableSceneNodesMessage.class));
}
