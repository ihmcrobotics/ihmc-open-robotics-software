package us.ihmc.perception.sceneGraph.ros2;

import org.apache.commons.lang.NotImplementedException;
import perception_msgs.msg.dds.DoorOpeningMechanismMessage;
import perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism.DoorOpeningMechanismType;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;

public class ROS2SceneGraphTools
{
   public static SceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode, SceneGraph sceneGraph)
   {
      SceneNode sceneNode;

      byte nodeType = subscriptionNode.getType();
      long nodeID = subscriptionNode.getSceneNodeMessage().getId();
      String nodeName = subscriptionNode.getSceneNodeMessage().getNameAsString();
      CRDTInfo crdtInfo = sceneGraph.getCRDTInfo();

      if (nodeType == SceneGraphMessage.PREDEFINED_RIGID_BODY_NODE_TYPE || nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
      {
         PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage = subscriptionNode.getPredefinedRigidBodySceneNodeMessage();
         RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getInitialTransformToParent(), initialTransformToParent);
         RigidBodyTransform visualTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getVisualTransformToParent(), visualTransformToParent);

         if (nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
         {
            sceneNode = new StaticRelativeSceneNode(nodeID,
                                                    nodeName,
                                                    sceneGraph.getIDToNodeMap(),
                                                    predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                    initialTransformToParent,
                                                    predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                    visualTransformToParent,
                                                    subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking(),
                                                    crdtInfo);
         }
         else // PREDEFINED_RIGID_BODY_NODE_TYPE
         {
            sceneNode = new PredefinedRigidBodySceneNode(nodeID,
                                                         nodeName,
                                                         sceneGraph.getIDToNodeMap(),
                                                         predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                         initialTransformToParent,
                                                         predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                         visualTransformToParent,
                                                         crdtInfo);
         }
      }
      else if (nodeType == SceneGraphMessage.ARUCO_MARKER_NODE_TYPE)
      {
         sceneNode = new ArUcoMarkerNode(nodeID,
                                         nodeName,
                                         subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId(),
                                         subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize(),
                                         crdtInfo);
      }
      else if (nodeType == SceneGraphMessage.CENTERPOSE_NODE_TYPE)
      {
         sceneNode = new CenterposeNode(nodeID,
                                        nodeName,
                                        CenterPoseInstantDetection.fromMessage(subscriptionNode.getCenterposeNodeMessage()
                                                                                               .getDetectableSceneNode()
                                                                                               .getLatestDetections()
                                                                                               .get(0)),
                                        subscriptionNode.getCenterposeNodeMessage().getEnableTracking(),
                                        crdtInfo);
      }
      else if (nodeType == SceneGraphMessage.YOLO_NODE_TYPE)
      {
         sceneNode = new YOLOv8Node(nodeID,
                                    nodeName,
                                    YOLOv8InstantDetection.fromMessage(subscriptionNode.getYOLONodeMessage()
                                                                                       .getDetectableSceneNode()
                                                                                       .getLatestDetections()
                                                                                       .get(0)),
                                    subscriptionNode.getYOLONodeMessage().getCentroidToObjectTransform(),
                                    subscriptionNode.getYOLONodeMessage().getObjectPose(),
                                    crdtInfo);
      }
      else if (nodeType == SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE)
      {
         throw new NotImplementedException("TOMASZ YOU LEFT THIS BUG IN THE CODE");
      }
      else if (nodeType == SceneGraphMessage.PRIMITIVE_RIGID_BODY_NODE_TYPE)
      {
         PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage = subscriptionNode.getPrimitiveRigidBodySceneNodeMessage();
         RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(primitiveRigidBodySceneNodeMessage.getInitialTransformToParent(), initialTransformToParent);
         sceneNode = new PrimitiveRigidBodySceneNode(nodeID,
                                                     nodeName,
                                                     sceneGraph.getIDToNodeMap(),
                                                     primitiveRigidBodySceneNodeMessage.getInitialParentId(),
                                                     initialTransformToParent,
                                                     PrimitiveRigidBodyShape.fromString(primitiveRigidBodySceneNodeMessage.getShapeAsString()),
                                                     crdtInfo);
      }
      else if (nodeType == SceneGraphMessage.DOOR_NODE_TYPE)
      {
         // TODO: DOORNODES
         DoorNode doorNode = new DoorNode(nodeID, crdtInfo);
         doorNode.getDoorFramePose().set(subscriptionNode.getDoorNodeMessage().getDoorFramePose());
         doorNode.getDoorPanel().fromMessage(subscriptionNode.getDoorNodeMessage().getDoorPanel());
         doorNode.getOpeningMechanisms().clear(); // TODO should we clear the list every time?
         for (DoorOpeningMechanismMessage doorOpeningMechanismMessage : subscriptionNode.getDoorNodeMessage().getOpeningMechanisms())
         {
            DoorSide doorSide = DoorSide.fromByte(doorOpeningMechanismMessage.getDoorSide());
            DoorOpeningMechanismType openingMechanismType = DoorOpeningMechanismType.fromByte(doorOpeningMechanismMessage.getType());
            DoorOpeningMechanism doorOpeningMechanism = new DoorOpeningMechanism(doorSide, openingMechanismType);
            doorOpeningMechanism.getGraspPose().set(doorOpeningMechanismMessage.getGraspPose());
            doorNode.getOpeningMechanisms().add(doorOpeningMechanism);
         }
         sceneNode = doorNode;
      }
      else
      {
         sceneNode = new SceneNode(nodeID, nodeName, crdtInfo);
      }

      sceneGraph.getIDToNodeMap().put(nodeID, sceneNode); // Make sure any new nodes are in the map // TODO: Probably remove not necessary

      return sceneNode;
   }
}
