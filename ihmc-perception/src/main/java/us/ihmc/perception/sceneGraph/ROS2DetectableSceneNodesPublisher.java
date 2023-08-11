package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

/**
 * Publishes the current state of the complete collection of detectable scene objects.
 * Publishing all scene objects in one message can simplify synchronization and
 * reduce the complexity of logic in figuring out when objects are currently under
 * consideraion.
 */
public class ROS2DetectableSceneNodesPublisher
{
   private final DetectableSceneNodesMessage detectableSceneNodesMessage = new DetectableSceneNodesMessage();
   private final FramePose3D sceneNodePose = new FramePose3D();
   private final FramePose3D arUcoMarkerPose = new FramePose3D();
   private final RigidBodyTransform sceneNodeToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform arUcoMarkerToWorldTransform = new RigidBodyTransform();

   /**
    * @param ioQualifier Depending whether on the robot or in some other process.
    *                    If in the on-robot perception process, STATUS, else COMMAND.
    */
   public void publish(Iterable<DetectableSceneNode> detectableSceneNodes,
                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                       ROS2IOTopicQualifier ioQualifier)
   {
      detectableSceneNodesMessage.getDetectableSceneNodes().clear();
      for (DetectableSceneNode detectableSceneNode : detectableSceneNodes)
      {
         DetectableSceneNodeMessage detectableSceneNodeMessage = detectableSceneNodesMessage.getDetectableSceneNodes().add();
         detectableSceneNodeMessage.setName(detectableSceneNode.getName());
         detectableSceneNodeMessage.setCurrentlyDetected(detectableSceneNode.getCurrentlyDetected());
         detectableSceneNodeMessage.setTrackDetectedPose(detectableSceneNode.getTrackDetectedPose());

         sceneNodePose.setToZero(detectableSceneNode.getNodeFrame());
         sceneNodePose.changeFrame(ReferenceFrame.getWorldFrame());
         sceneNodePose.get(sceneNodeToWorldTransform);
         MessageTools.toMessage(sceneNodeToWorldTransform, detectableSceneNodeMessage.getTransformToWorld());

         // We must syncronize the ArUco marker frames so we can reset overriden node
         // poses back to ArUco relative ones.
         if (detectableSceneNode instanceof ArUcoDetectableNode arUcoDetectableNode)
         {
            arUcoMarkerPose.setToZero(arUcoDetectableNode.getMarkerFrame());
            arUcoMarkerPose.changeFrame(ReferenceFrame.getWorldFrame());
            arUcoMarkerPose.get(arUcoMarkerToWorldTransform);
            MessageTools.toMessage(arUcoMarkerToWorldTransform, detectableSceneNodeMessage.getArucoMarkerTransformToWorld());

            detectableSceneNodeMessage.setBreakFrequency((float) arUcoDetectableNode.getBreakFrequency());
         }
         if (detectableSceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
         {
            detectableSceneNodeMessage.setDistanceToDisableTracking((float) staticRelativeNode.getDistanceToDisableTracking());
            detectableSceneNodeMessage.setCurrentDistanceToRobot((float) staticRelativeNode.getCurrentDistance());
         }
      }
      ros2PublishSubscribeAPI.publish(PerceptionAPI.DETECTABLE_SCENE_NODES.getTopic(ioQualifier), detectableSceneNodesMessage);
   }
}
