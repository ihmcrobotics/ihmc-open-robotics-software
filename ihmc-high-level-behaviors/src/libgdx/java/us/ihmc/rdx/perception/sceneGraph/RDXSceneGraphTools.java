package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphTools;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXSceneGraphTools
{
   public static RDXSceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode, RDX3DPanel panel3D, SceneGraph sceneGraph)
   {
      SceneNode sceneNode = ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph);

      RDXSceneNode uiSceneNode;
      if (sceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         uiSceneNode = new RDXArUcoMarkerNode(arUcoMarkerNode);
      }
      else if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         uiSceneNode = new RDXStaticRelativeSceneNode(staticRelativeSceneNode, panel3D);
      }
      else if (sceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
      {
         uiSceneNode = new RDXPredefinedRigidBodySceneNode(predefinedRigidBodySceneNode, panel3D);
      }
      else if (sceneNode instanceof DetectableSceneNode detectableSceneNode)
      {
         uiSceneNode = new RDXDetectableSceneNode(detectableSceneNode);
      }
      else
      {
         uiSceneNode = new RDXSceneNode(sceneNode);
      }
      return uiSceneNode;
   }
}
