package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;

import java.util.List;

/**
 * Subscribes to, synchronizing, a list of detectable nodes in the robot's perception scene graph.
 */
public class ROS2DetectableSceneNodesSubscription
{
   private final IHMCROS2Input<DetectableSceneNodesMessage> detectableSceneNodesSubscription;
   private final List<DetectableSceneNode> detectableSceneNodes;
   private final FramePose3D nodePose = new FramePose3D();
   private final FramePose3D arUcoMarkerPose = new FramePose3D();
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform arUcoMarkerToWorldTransform = new RigidBodyTransform();
   private long numberOfMessagesReceived = 0;

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    */
   public ROS2DetectableSceneNodesSubscription(List<DetectableSceneNode> detectableSceneNodes,
                                               ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                               ROS2IOTopicQualifier ioQualifier)
   {
      this.detectableSceneNodes = detectableSceneNodes;

      detectableSceneNodesSubscription = ros2PublishSubscribeAPI.subscribe(PerceptionAPI.DETECTABLE_SCENE_NODES.getTopic(ioQualifier));
   }

   /**
    * Check for a new ROS 2 message and update the scene nodes with it.
    * This method runs on the robot and on every connected UI.
    * @return if a new message was used to update the scene nodes on this call
    */
   public boolean update()
   {
      boolean newMessageAvailable = detectableSceneNodesSubscription.getMessageNotification().poll();
      if (newMessageAvailable)
      {
         ++numberOfMessagesReceived;
         DetectableSceneNodesMessage detectableSceneNodesMessage = detectableSceneNodesSubscription.getMessageNotification().read();
         IDLSequence.Object<DetectableSceneNodeMessage> detectableSceneNodeMessages = detectableSceneNodesMessage.getDetectableSceneNodes();

         // We group this because with a tree structure, modification propagate.
         // Maybe we should build that in at some point instead of freezing the whole tree,
         // but it's good enough for now.
         boolean operatorHasntModifiedAnythingRecently = true;
         for (DetectableSceneNode detectableSceneNode : detectableSceneNodes)
         {
            operatorHasntModifiedAnythingRecently &= detectableSceneNode.operatorHasntModifiedThisRecently();
         }

         // We assume the nodes are in the same order on both sides. This is to avoid O(n^2) complexity.
         // We could improve on this design later.
         for (int i = 0; i < detectableSceneNodeMessages.size(); i++)
         {
            DetectableSceneNodeMessage detectableSceneNodeMessage = detectableSceneNodesMessage.getDetectableSceneNodes().get(i);
            DetectableSceneNode detectableSceneNode = detectableSceneNodes.get(i);

            if (detectableSceneNodeMessage.getNameAsString().equals(detectableSceneNode.getName()))
            {
               detectableSceneNode.setCurrentlyDetected(detectableSceneNodeMessage.getCurrentlyDetected());

               // We must synchronize the ArUco marker frames so we can reset overriden node
               // poses back to ArUco relative ones.
               // The ArUco frame is the parent so we should update it first.
               if (detectableSceneNode instanceof ArUcoDetectableNode arUcoDetectableNode)
               {
                  MessageTools.toEuclid(detectableSceneNodeMessage.getArucoMarkerTransformToWorld(), arUcoMarkerToWorldTransform);
                  arUcoMarkerPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), arUcoMarkerToWorldTransform);
                  arUcoMarkerPose.changeFrame(arUcoDetectableNode.getMarkerFrame().getParent());
                  arUcoMarkerPose.get(arUcoDetectableNode.getMarkerToWorldFrameTransform());
                  arUcoDetectableNode.getMarkerFrame().update();
               }

               // If the node was recently modified by the operator, the node does not accept
               // updates of the "track detected pose" setting. This is to allow the operator's changes to propagate
               // and so it doesn't get overriden immediately by an out of date message coming from the robot.
               // On the robot side, this will always get updated because there is no operator.
               if (operatorHasntModifiedAnythingRecently)
               {
                  detectableSceneNode.setTrackDetectedPose(detectableSceneNodeMessage.getTrackDetectedPose());
                  if (detectableSceneNode instanceof ArUcoDetectableNode arUcoDetectableNode)
                  {
                     arUcoDetectableNode.setBreakFrequency(detectableSceneNodeMessage.getBreakFrequency());
                  }

                  MessageTools.toEuclid(detectableSceneNodeMessage.getTransformToWorld(), nodeToWorldTransform);
                  nodePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), nodeToWorldTransform);
                  nodePose.changeFrame(detectableSceneNode.getNodeFrame().getParent());
                  nodePose.get(detectableSceneNode.getNodeToParentFrameTransform());
                  detectableSceneNode.getNodeFrame().update();
               }
            }
            else
            {
               LogTools.warn("Scene graph nodes are out of sync. %s != %s".formatted(detectableSceneNodeMessage.getNameAsString(),
                                                                                     detectableSceneNode.getName()));
            }
         }
      }
      return newMessageAvailable;
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }
}
