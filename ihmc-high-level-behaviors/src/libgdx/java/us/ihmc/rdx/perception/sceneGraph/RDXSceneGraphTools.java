package us.ihmc.rdx.perception.sceneGraph;

import perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXSceneGraphTools
{
   public static SceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode, RDX3DPanel panel3D, SceneGraph sceneGraph)
   {
      SceneNode sceneNode;

      byte nodeType = subscriptionNode.getType();
      long nodeID = subscriptionNode.getSceneNodeMessage().getId();
      String nodeName = subscriptionNode.getSceneNodeMessage().getNameAsString();

      if (nodeType == SceneGraphMessage.PREDEFINED_RIGID_BODY_NODE_TYPE || nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
      {
         PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage = subscriptionNode.getPredefinedRigidBodySceneNodeMessage();
         RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getInitialTransformToParent(), initialTransformToParent);
         RigidBodyTransform visualTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getVisualTransformToParent(), visualTransformToParent);

         if (nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
         {
            sceneNode = new RDXStaticRelativeSceneNode(nodeID,
                                                       nodeName,
                                                       sceneGraph.getIDToNodeMap(),
                                                       predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                       initialTransformToParent,
                                                       predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                       visualTransformToParent,
                                                       subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking(),
                                                       panel3D);
         }
         else // PREDEFINED_RIGID_BODY_NODE_TYPE
         {
            sceneNode = new RDXPredefinedRigidBodySceneNode(nodeID,
                                                            nodeName,
                                                            sceneGraph.getIDToNodeMap(),
                                                            predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                            initialTransformToParent,
                                                            predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                            visualTransformToParent,
                                                            panel3D);
         }
      }
      else if (nodeType == SceneGraphMessage.ARUCO_MARKER_NODE_TYPE)
      {
         sceneNode = new RDXArUcoMarkerNode(nodeID,
                                            nodeName,
                                            subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId(),
                                            subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize());
      }
      else if (nodeType == SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE)
      {
         sceneNode = new RDXDetectableSceneNode(nodeID, nodeName);
      }
      else
      {
         sceneNode = new RDXSceneNode(nodeID, nodeName);
      }

      return sceneNode;
   }
}
