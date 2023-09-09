package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneNode;

public class ROS2SceneGraphTools
{
   public static SceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      SceneNode sceneNode = null;
      DetectableSceneNode detectableSceneNode = null;


      if (subscriptionNode.getType() == SceneGraphMessage.ARUCO_MARKER_NODE_TYPE)
      {
//         new ArUcoMarkerNode(); TODO
      }

      if (subscriptionNode.getType() == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
      {
//         new StaticRelativeSceneNode(); TODO
      }

      if (subscriptionNode.getType() == SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE)
      {
         detectableSceneNode = new DetectableSceneNode(subscriptionNode.getSceneNodeMessage().getId(),
                                                       subscriptionNode.getSceneNodeMessage().getNameAsString());
         detectableSceneNode.setCurrentlyDetected(subscriptionNode.getDetectableSceneNodeMessage().getCurrentlyDetected());
         sceneNode = detectableSceneNode;
      }

      if (sceneNode == null)
      {
         sceneNode = new SceneNode(subscriptionNode.getSceneNodeMessage().getId(), subscriptionNode.getSceneNodeMessage().getNameAsString());
      }

      return sceneNode;
   }
}
