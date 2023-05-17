package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;

import java.util.List;

/**
 * Subscribes to, synchronizing, a list of detectable nodes in the robot's perception scene graph.
 */
public class ROS2DetectableSceneNodesSubscription
{
   private final IHMCROS2Input<DetectableSceneNodesMessage> detectableSceneNodesSubscription;
   private final ROS2IOTopicQualifier ioQualifier;
   private final List<DetectableSceneNode> detectableSceneNodes;
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private final FramePose3D nodePose = new FramePose3D();
   private long numberOfMessagesReceived = 0;

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    */
   public ROS2DetectableSceneNodesSubscription(List<DetectableSceneNode> detectableSceneNodes,
                                               ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                               ROS2IOTopicQualifier ioQualifier)
   {
      this.detectableSceneNodes = detectableSceneNodes;
      this.ioQualifier = ioQualifier;

      detectableSceneNodesSubscription = ros2PublishSubscribeAPI.subscribe(SceneGraphAPI.DETECTABLE_SCENE_NODES.getTopic(ioQualifier));
   }

   /**
    * Check for a new ROS 2 message and update the scene nodes with it.
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

         // We assume the nodes are in the same order on both sides. This is to avoid O(n^2) complexity.
         // We could improve on this design later.
         for (int i = 0; i < detectableSceneNodeMessages.size(); i++)
         {
            DetectableSceneNodeMessage detectableSceneNodeMessage = detectableSceneNodesMessage.getDetectableSceneNodes().get(i);
            DetectableSceneNode detectableSceneNode = detectableSceneNodes.get(i);

            if (detectableSceneNodeMessage.getNameAsString().equals(detectableSceneNode.getName()))
            {
               detectableSceneNode.setCurrentlyDetected(detectableSceneNodeMessage.getCurrentlyDetected());

               // We allow only the operator to change the "poseOverriddenByOperator" variable,
               // to avoid conflicts and race conditions
               boolean isOperatorSide = ioQualifier == ROS2IOTopicQualifier.STATUS;
               boolean isOnRobot = ioQualifier == ROS2IOTopicQualifier.COMMAND;
               if (isOnRobot)
               {
                  // We'll always take the state of the operator's scene for this variable
                  detectableSceneNode.setPoseOverriddenByOperator(detectableSceneNodeMessage.getIsPoseOverriddenByOperator());
               }

               // If on robot perception process and pose overridden by operator, accept the pose
               // else, if on operator side and pose not overridden by operator, accept the pose
               boolean acceptPoseUpdate = isOnRobot && detectableSceneNode.getPoseOverriddenByOperator();
               acceptPoseUpdate |= isOperatorSide && !detectableSceneNode.getPoseOverriddenByOperator();
               if (acceptPoseUpdate)
               {
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
