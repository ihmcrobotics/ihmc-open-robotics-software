package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.*;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

/**
 * Publishes the current state of the complete scene graph.
 * Publishing all scene objects in one message can simplify synchronization and
 * reduce the complexity of logic in figuring out when objects are currently under
 * consideration.
 */
public class ROS2SceneGraphPublisher
{
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
      sceneGraphMessage.setNextId(SceneGraph.NEXT_ID.intValue());
      sceneGraphMessage.getSceneTreeTypes().clear();
      sceneGraphMessage.getSceneTreeIndices().clear();
      sceneGraphMessage.getSceneNodes().clear();
      sceneGraphMessage.getDetectableSceneNodes().clear();
      sceneGraphMessage.getArucoMarkerSceneNodes().clear();
      sceneGraphMessage.getStaticRelativeSceneNodes().clear();

      packSceneTreeToMessage(sceneGraph.getRootNode(), sceneGraphMessage);

      ros2PublishSubscribeAPI.publish(PerceptionAPI.SCENE_GRAPH.getTopic(ioQualifier), sceneGraphMessage);
   }

   private void packSceneTreeToMessage(SceneNode sceneNode, SceneGraphMessage sceneGraphMessage)
   {
      SceneNodeMessage sceneNodeMessage = null;
      DetectableSceneNodeMessage detectableSceneNodeMessage = null;

      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE);
         sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getStaticRelativeSceneNodes().size());
         StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage = sceneGraphMessage.getStaticRelativeSceneNodes().add();
         staticRelativeSceneNodeMessage.setDistanceToDisableTracking((float) staticRelativeSceneNode.getDistanceToDisableTracking());
         staticRelativeSceneNodeMessage.setCurrentDistanceToRobot((float) staticRelativeSceneNode.getCurrentDistance());
         sceneNodeMessage = staticRelativeSceneNodeMessage.getSceneNode();
      }

      if (sceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.ARUCO_MARKER_NODE_TYPE);
         sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getArucoMarkerSceneNodes().size());
         ArUcoMarkerNodeMessage arUcoMarkerNodeMessage = sceneGraphMessage.getArucoMarkerSceneNodes().add();
         arUcoMarkerNodeMessage.setBreakFrequency((float) arUcoMarkerNode.getBreakFrequency());
         detectableSceneNodeMessage = arUcoMarkerNodeMessage.getDetectableSceneNode();
      }

      if (sceneNode instanceof DetectableSceneNode detectableSceneNode)
      {
         if (detectableSceneNodeMessage == null)
         {
            sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE);
            sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getDetectableSceneNodes().size());
            detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().add();
         }

         detectableSceneNodeMessage.setCurrentlyDetected(detectableSceneNode.getCurrentlyDetected());
         sceneNodeMessage = detectableSceneNodeMessage.getSceneNode();
      }

      if (sceneNodeMessage == null)
      {
         sceneGraphMessage.getSceneTreeTypes().add(SceneGraphMessage.SCENE_NODE_TYPE);
         sceneGraphMessage.getSceneTreeIndices().add(sceneGraphMessage.getSceneNodes().size());
         sceneNodeMessage = sceneGraphMessage.getSceneNodes().add();
      }

      sceneNodeMessage.setName(sceneNode.getName());
      sceneNodePose.setToZero(sceneNode.getNodeFrame());
      sceneNodePose.changeFrame(ReferenceFrame.getWorldFrame());
      sceneNodePose.get(sceneNodeToWorldTransform);
      MessageTools.toMessage(sceneNodeToWorldTransform, sceneNodeMessage.getTransformToWorld());

      for (SceneNode child : sceneNode.getChildren())
      {
         packSceneTreeToMessage(child, sceneGraphMessage);
      }
   }
}
