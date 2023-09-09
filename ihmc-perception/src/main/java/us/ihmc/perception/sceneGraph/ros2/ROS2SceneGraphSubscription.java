package us.ihmc.perception.sceneGraph.ros2;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import perception_msgs.msg.dds.ArUcoMarkerNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import perception_msgs.msg.dds.StaticRelativeSceneNodeMessage;
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
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

/**
 * Subscribes to, synchronizing, the robot's perception scene graph.
 */
public class ROS2SceneGraphSubscription
{
   private final IHMCROS2Input<SceneGraphMessage> sceneGraphSubscription;
   private final SceneGraph sceneGraph;
   private final FramePose3D nodePose = new FramePose3D();
   private final FramePose3D arUcoMarkerPose = new FramePose3D();
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform arUcoMarkerToWorldTransform = new RigidBodyTransform();
   private long numberOfMessagesReceived = 0;
   private final TLongObjectMap<SceneNode> disassembledTreeMap = new TLongObjectHashMap<>();

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    */
   public ROS2SceneGraphSubscription(SceneGraph sceneGraph,
                                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2IOTopicQualifier ioQualifier)
   {
      this.sceneGraph = sceneGraph;

      sceneGraphSubscription = ros2PublishSubscribeAPI.subscribe(PerceptionAPI.SCENE_GRAPH.getTopic(ioQualifier));
   }

   /**
    * Check for a new ROS 2 message and update the scene nodes with it.
    * This method runs on the robot and on every connected UI.
    * @return if a new message was used to update the scene nodes on this call
    */
   public boolean update()
   {
      boolean newMessageAvailable = sceneGraphSubscription.getMessageNotification().poll();
      if (newMessageAvailable)
      {
         ++numberOfMessagesReceived;
         SceneGraphMessage sceneGraphMessage = sceneGraphSubscription.getMessageNotification().read();
         IDLSequence.Object<DetectableSceneNodeMessage> detectableSceneNodeMessages = sceneGraphMessage.getDetectableSceneNodes();

         ROS2SceneGraphSubscriptionNode subscriptionNodeTree = new ROS2SceneGraphSubscriptionNode();
         buildSubscriptionTree(0, sceneGraphMessage, subscriptionNodeTree);

         disassembleLocalTree(sceneGraph.getRootNode());

         updateLocalTreeFromSubscription(subscriptionNodeTree, sceneGraph.getRootNode());

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
            DetectableSceneNodeMessage detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().get(i);
            DetectableSceneNode detectableSceneNode = detectableSceneNodes.get(i);

            if (detectableSceneNodeMessage.getNameAsString().equals(detectableSceneNode.getName()))
            {
               detectableSceneNode.setCurrentlyDetected(detectableSceneNodeMessage.getCurrentlyDetected());

               // We must synchronize the ArUco marker frames so we can reset overriden node
               // poses back to ArUco relative ones.
               // The ArUco frame is the parent so we should update it first.
               if (detectableSceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
               {
                  MessageTools.toEuclid(detectableSceneNodeMessage.getArucoMarkerTransformToWorld(), arUcoMarkerToWorldTransform);
                  arUcoMarkerPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), arUcoMarkerToWorldTransform);
                  arUcoMarkerPose.changeFrame(arUcoMarkerNode.getNodeFrame().getParent());
                  arUcoMarkerPose.get(arUcoMarkerNode.getNodeToParentFrameTransform());
                  arUcoMarkerNode.getNodeFrame().update();
               }
               if (detectableSceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
               {
                  staticRelativeNode.setCurrentDistance(detectableSceneNodeMessage.getCurrentDistanceToRobot());
               }

               // If the node was recently modified by the operator, the node does not accept
               // updates of the "track detected pose" setting. This is to allow the operator's changes to propagate
               // and so it doesn't get overriden immediately by an out of date message coming from the robot.
               // On the robot side, this will always get updated because there is no operator.
               if (operatorHasntModifiedAnythingRecently)
               {
                  detectableSceneNode.setTrackDetectedPose(detectableSceneNodeMessage.getTrackDetectedPose());
                  if (detectableSceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
                  {
                     arUcoMarkerNode.setBreakFrequency(detectableSceneNodeMessage.getBreakFrequency());
                  }
                  if (detectableSceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
                  {
                     staticRelativeNode.setDistanceToDisableTracking(detectableSceneNodeMessage.getDistanceToDisableTracking());
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

   private void updateLocalTreeFromSubscription(ROS2SceneGraphSubscriptionNode subscriptionNode, SceneNode localNode)
   {
      for (ROS2SceneGraphSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         SceneNode localChildNode = disassembledTreeMap.get(subscriptionChildNode.getSceneNodeMessage().getId());

         if (localChildNode == null) // New node that wasn't in the local tree
         {
            localChildNode = ROS2SceneGraphTools.createNodeFromMessage(subscriptionChildNode);
         }





         localNode.getChildren().add(localChildNode);
         updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode);
      }
   }

   private void disassembleLocalTree(SceneNode localNode)
   {
      disassembledTreeMap.put(localNode.getID(), localNode);

      for (SceneNode child : localNode.getChildren())
      {
         disassembleLocalTree(child);
      }

      localNode.getChildren().clear();
   }

   private void buildSubscriptionTree(int index, SceneGraphMessage sceneGraphMessage, ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      byte sceneNodeType = sceneGraphMessage.getSceneTreeTypes().get(index);
      int indexInTypesList = (int) sceneGraphMessage.getSceneTreeIndices().get(index);
      switch (sceneNodeType)
      {
         case SceneGraphMessage.SCENE_NODE_TYPE ->
               subscriptionNode.setSceneNodeMessage(sceneGraphMessage.getSceneNodes().get(indexInTypesList));
         case SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE ->
         {
            DetectableSceneNodeMessage detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().get(indexInTypesList);
            subscriptionNode.setDetectableSceneNodeMessage(detectableSceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(detectableSceneNodeMessage.getSceneNode());
         }
         case SceneGraphMessage.ARUCO_MARKER_NODE_TYPE ->
         {
            ArUcoMarkerNodeMessage arUcoMarkerNodeMessage = sceneGraphMessage.getArucoMarkerSceneNodes().get(indexInTypesList);
            subscriptionNode.setArUcoMarkerNodeMessage(arUcoMarkerNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(arUcoMarkerNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(arUcoMarkerNodeMessage.getDetectableSceneNode().getSceneNode());
         }
         case SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE ->
         {
            StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage = sceneGraphMessage.getStaticRelativeSceneNodes().get(indexInTypesList);
            subscriptionNode.setStaticRelativeSceneNodeMessageeMessage(staticRelativeSceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(staticRelativeSceneNodeMessage.getSceneNode());
         }
      }

      for (int i = 0; i < subscriptionNode.getSceneNodeMessage().getNumberOfChildren(); i++)
      {
         ROS2SceneGraphSubscriptionNode subscriptionTreeChildNode = new ROS2SceneGraphSubscriptionNode();
         buildSubscriptionTree(index++, sceneGraphMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
   }

   public void destroy()
   {
      sceneGraphSubscription.destroy();
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }
}
