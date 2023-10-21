package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphTools;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXSceneGraphTools
{
   public static RDXSceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode, RDX3DPanel panel3D, SceneGraph sceneGraph)
   {
      // We create one using this and copy to save on code maintenance
      SceneNode sceneNodeToCopy = ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph);

      if (sceneNodeToCopy instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         return new RDXArUcoMarkerNode(arUcoMarkerNode);
      }
      else if (sceneNodeToCopy instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         return new RDXStaticRelativeSceneNode(staticRelativeSceneNode, panel3D);
      }
      else if (sceneNodeToCopy instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
      {
         return new RDXPredefinedRigidBodySceneNode(predefinedRigidBodySceneNode, panel3D);
      }
      else if (sceneNodeToCopy instanceof PrimitiveRigidBodySceneNode resizableRigidBodySceneNode)
      {
         return new RDXPrimitiveRigidBodySceneNode(resizableRigidBodySceneNode, panel3D);
      }
      else
      {
         return new RDXSceneNode(sceneNodeToCopy);
      }
   }
}
