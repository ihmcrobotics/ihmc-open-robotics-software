package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.couch.CouchNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.trashcan.TrashCanNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphTools;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXSceneGraphTools
{
   public static RDXSceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode,
                                                    RDX3DPanel panel3D,
                                                    ImGuiUniqueLabelMap labels,
                                                    SceneGraph sceneGraph)
   {
      // We create one using this and copy to save on code maintenance
      SceneNode sceneNodeToCopy = ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph);

      if (sceneNodeToCopy instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         return new RDXArUcoMarkerNode(arUcoMarkerNode);
      }
      else if (sceneNodeToCopy instanceof CenterposeNode centerposeNode)
      {
         return new RDXCenterposeNode(centerposeNode, panel3D);
      }
      else if (sceneNodeToCopy instanceof YOLOv8Node yoloNode)
      {
         return new RDXYOLOv8Node(yoloNode, labels);
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
      else if (sceneNodeToCopy instanceof DoorNode doorNode)
      {
         return new RDXDoorNode(doorNode);
      }
      else if (sceneNodeToCopy instanceof TrashCanNode trashCanNode)
      {
         return new RDXTrashCanNode(trashCanNode);
      }
      else if (sceneNodeToCopy instanceof CouchNode couchNode)
      {
         return new RDXCouchNode(couchNode);
      }
      else
      {
         return new RDXSceneNode(sceneNodeToCopy);
      }
   }
}
