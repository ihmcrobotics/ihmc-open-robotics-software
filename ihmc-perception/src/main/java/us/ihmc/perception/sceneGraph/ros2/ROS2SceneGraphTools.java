package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

public class ROS2SceneGraphTools
{
   public static SceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      return switch (subscriptionNode.getType())
      {
         case SceneGraphMessage.ARUCO_MARKER_NODE_TYPE -> new ArUcoMarkerNode(subscriptionNode.getSceneNodeMessage().getId(),
                                                                              subscriptionNode.getSceneNodeMessage().getNameAsString(),
                                                                              subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId(),
                                                                              subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize());
         case SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE ->
         {
            new StaticRelativeSceneNode(subscriptionNode.getSceneNodeMessage().getId(),
                                        subscriptionNode.getSceneNodeMessage().getNameAsString(),
                                        rootNodeSupplier,
                                        initialParentNodeSupplier,
                                        initialTransformToParent,
                                        visualModelFilePath,
                                        visualModelToNodeFrameTransform,
                                        subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking());
         }
         case SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE ->
               new DetectableSceneNode(subscriptionNode.getSceneNodeMessage().getId(), subscriptionNode.getSceneNodeMessage().getNameAsString());
         case SceneGraphMessage.SCENE_NODE_TYPE ->
               new SceneNode(subscriptionNode.getSceneNodeMessage().getId(), subscriptionNode.getSceneNodeMessage().getNameAsString());
         default -> null;
      };
   }
}
