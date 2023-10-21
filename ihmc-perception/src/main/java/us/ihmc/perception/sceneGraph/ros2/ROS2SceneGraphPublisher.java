package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.*;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;

/**
 * Publishes the current state of the complete scene graph.
 * Publishing all scene objects in one message can simplify synchronization and
 * reduce the complexity of logic in figuring out when objects are currently under
 * consideration.
 */
public class ROS2SceneGraphPublisher
{
   private ROS2IOTopicQualifier ioQualifier;
   private final SceneGraphMessage sceneGraphMessage = new SceneGraphMessage();
   private final FramePose3D sceneNodePose = new FramePose3D();
   private final RigidBodyTransform sceneNodeToWorldTransform = new RigidBodyTransform();

   /**
    * @param ioQualifier Depending whether on the robot or in some other process.
    *                    If in the on-robot perception process, STATUS, else COMMAND.
    */
   public void publish(SceneGraph sceneGraph,
                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                       ROS2IOTopicQualifier ioQualifier)
   {
      this.ioQualifier = ioQualifier;

      sceneGraphMessage.setNextId(sceneGraph.getNextID().intValue());
      sceneGraphMessage.getSceneTreeTypes().clear();
      sceneGraphMessage.getSceneTreeIndices().clear();
      sceneGraphMessage.getSceneNodes().clear();
      sceneGraphMessage.getDetectableSceneNodes().clear();
      sceneGraphMessage.getPredefinedRigidBodySceneNodes().clear();
      sceneGraphMessage.getArucoMarkerSceneNodes().clear();
      sceneGraphMessage.getStaticRelativeSceneNodes().clear();
      sceneGraphMessage.getPrimitiveRigidBodySceneNodes().clear();

      packSceneTreeToMessage(sceneGraph.getRootNode(), sceneGraphMessage);

      ros2PublishSubscribeAPI.publish(PerceptionAPI.SCENE_GRAPH.getTopic(ioQualifier), sceneGraphMessage);
   }

   private void packSceneTreeToMessage(SceneNode sceneNode, SceneGraphMessage sceneGraphMessage)
   {
      // We handle packing the most specific types and then the more basic ones
      // We keep track of the more basic ones so we can share the
      // packing logic below.
      SceneNodeMessage sceneNodeMessage;

      if (sceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
      {
         PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage;
         if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
         {
            sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE);
            sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getStaticRelativeSceneNodes().size());
            StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage = sceneGraphMessage.getStaticRelativeSceneNodes().add();
            staticRelativeSceneNodeMessage.setDistanceToDisableTracking((float) staticRelativeSceneNode.getDistanceToDisableTracking());
            staticRelativeSceneNodeMessage.setCurrentDistanceToRobot((float) staticRelativeSceneNode.getCurrentDistance());
            predefinedRigidBodySceneNodeMessage = staticRelativeSceneNodeMessage.getPredefinedRigidBodySceneNode();
         }
         else
         {
            sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.PREDEFINED_RIGID_BODY_NODE_TYPE);
            sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getPredefinedRigidBodySceneNodes().size());
            predefinedRigidBodySceneNodeMessage = sceneGraphMessage.getPredefinedRigidBodySceneNodes().add();
         }

         predefinedRigidBodySceneNodeMessage.setInitialParentId(predefinedRigidBodySceneNode.getInitialParentNodeID());
         MessageTools.toMessage(predefinedRigidBodySceneNode.getInitialTransformToParent(),
                                predefinedRigidBodySceneNodeMessage.getInitialTransformToParent());
         predefinedRigidBodySceneNodeMessage.setVisualModelFilePath(predefinedRigidBodySceneNode.getVisualModelFilePath());
         MessageTools.toMessage(predefinedRigidBodySceneNode.getVisualModelToNodeFrameTransform(),
                                predefinedRigidBodySceneNodeMessage.getVisualTransformToParent());
         sceneNodeMessage = predefinedRigidBodySceneNodeMessage.getSceneNode();
      }
      else if (sceneNode instanceof DetectableSceneNode detectableSceneNode)
      {
         DetectableSceneNodeMessage detectableSceneNodeMessage;
         if (sceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
         {
            sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.ARUCO_MARKER_NODE_TYPE);
            sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getArucoMarkerSceneNodes().size());
            ArUcoMarkerNodeMessage arUcoMarkerNodeMessage = sceneGraphMessage.getArucoMarkerSceneNodes().add();
            arUcoMarkerNodeMessage.setMarkerId(arUcoMarkerNode.getMarkerID());
            arUcoMarkerNodeMessage.setMarkerSize((float) arUcoMarkerNode.getMarkerSize());
            arUcoMarkerNodeMessage.setBreakFrequency((float) arUcoMarkerNode.getBreakFrequency());
            detectableSceneNodeMessage = arUcoMarkerNodeMessage.getDetectableSceneNode();
         }
         else
         {
            sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE);
            sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getDetectableSceneNodes().size());
            detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().add();
         }

         detectableSceneNodeMessage.setCurrentlyDetected(detectableSceneNode.getCurrentlyDetected());
         sceneNodeMessage = detectableSceneNodeMessage.getSceneNode();
      }
      else if (sceneNode instanceof PrimitiveRigidBodySceneNode reshapableRigidBodySceneNode)
      {
         sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.PRIMITIVE_RIGID_BODY_NODE_TYPE);
         sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getPrimitiveRigidBodySceneNodes().size());
         PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage = sceneGraphMessage.getPrimitiveRigidBodySceneNodes().add();

         primitiveRigidBodySceneNodeMessage.setInitialParentId(reshapableRigidBodySceneNode.getInitialParentNodeID());
         primitiveRigidBodySceneNodeMessage.setShape(reshapableRigidBodySceneNode.getShape().name());
         MessageTools.toMessage(reshapableRigidBodySceneNode.getInitialTransformToParent(),
                                primitiveRigidBodySceneNodeMessage.getInitialTransformToParent());
         sceneNodeMessage = primitiveRigidBodySceneNodeMessage.getSceneNode();
      }
      else // In this case the node is just the most basic type
      {
         sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.SCENE_NODE_TYPE);
         sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getSceneNodes().size());
         sceneNodeMessage = sceneGraphMessage.getSceneNodes().add();
      }

      sceneNodeMessage.setId(sceneNode.getID());
      sceneNodeMessage.setName(sceneNode.getName());
      sceneNodePose.setToZero(sceneNode.getNodeFrame());
      sceneNodePose.changeFrame(ReferenceFrame.getWorldFrame());
      sceneNodePose.get(sceneNodeToWorldTransform);
      MessageTools.toMessage(sceneNodeToWorldTransform, sceneNodeMessage.getTransformToWorld());
      sceneNodeMessage.setNumberOfChildren(sceneNode.getChildren().size());

      for (SceneNode child : sceneNode.getChildren())
      {
         packSceneTreeToMessage(child, sceneGraphMessage);
      }
   }
}
