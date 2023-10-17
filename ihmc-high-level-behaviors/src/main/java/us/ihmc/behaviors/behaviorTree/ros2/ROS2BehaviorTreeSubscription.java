package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeReplacement;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;

import java.util.function.BiFunction;

public class ROS2BehaviorTreeSubscription
{
   private final IHMCROS2Input<BehaviorTreeStateMessage> behaviorTreeSubscription;
   private final BehaviorTreeState behaviorTree;
   private long numberOfMessagesReceived = 0;
   private boolean localTreeFrozen = false;
   private BehaviorTreeStateMessage latestBehaviorTreeMessage;

   /**
    * @param ioQualifier If in the on-robot autonomy process, COMMAND, else STATUS
    * @param newNodeSupplier So that new nodes can be externally extended, like for UI representations.
    *                        Use {@link ROS2BehaviorTreeTools#createNodeFromMessage} as a base.
    */
   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTree,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                       ROS2IOTopicQualifier ioQualifier)
   {
      this.behaviorTree = behaviorTree;

      behaviorTreeSubscription = ros2PublishSubscribeAPI.subscribe(AutonomyAPI.BEAVIOR_TREE.getTopic(ioQualifier));
   }

   public void update()
   {
      boolean newMessageAvailable = behaviorTreeSubscription.getMessageNotification().poll();
      if (newMessageAvailable)
      {
         ++numberOfMessagesReceived;
         latestBehaviorTreeMessage = behaviorTreeSubscription.getMessageNotification().read();

         subscriptionRootNode.clear();
         subscriptionNodeDepthFirstIndex.setValue(0);
         buildSubscriptionTree(latestBehaviorTreeMessage, subscriptionRootNode);

         // If the tree was recently modified by the operator, we do not accept
         // updates the structure of the tree.
         localTreeFrozen = false;
         checkTreeModified(sceneGraph.getRootNode());

         if (!localTreeFrozen)
            behaviorTree.getNextID().setValue(latestBehaviorTreeMessage.getNextId());

         behaviorTree.modifyTree(modificationQueue ->
         {
            if (!localTreeFrozen)
               modificationQueue.accept(new SceneGraphClearSubtree(behaviorTree.getRootNode()));

            updateLocalTreeFromSubscription(subscriptionRootNode, behaviorTree.getRootNode(), null, modificationQueue);
         });
      }
      return newMessageAvailable;
   }


   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                BehaviorTreeNodeState localNode,
                                                BehaviorTreeNodeState localParentNode,
                                                SceneGraphModificationQueue modificationQueue)
   {
      // If tree is frozen and the ID isn't in the local tree, we don't have anything to update
      // This'll happen if a node is deleted locally. We want that change to propagate and not add
      // it right back.
      if (localNode instanceof DetectableSceneNode detectableSceneNode)
      {
         detectableSceneNode.setCurrentlyDetected(subscriptionNode.getDetectableSceneNodeMessage().getCurrentlyDetected());
      }
      if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.setCurrentDistance(subscriptionNode.getStaticRelativeSceneNodeMessage().getCurrentDistanceToRobot());
      }

      // If the node was recently modified by the operator, the node does not accept
      // updates of these values. This is to allow the operator's changes to propagate
      // and so it doesn't get overriden immediately by an out of date message coming from the robot.
      // On the robot side, this will always get updated because there is no operator.
      if (!localTreeFrozen)
      {
         if (localNode instanceof ArUcoMarkerNode arUcoMarkerNode)
         {
            arUcoMarkerNode.setMarkerID(subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId());
            arUcoMarkerNode.setMarkerSize(subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize());
            arUcoMarkerNode.setBreakFrequency(subscriptionNode.getArUcoMarkerNodeMessage().getBreakFrequency());
         }
         if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
         {
            staticRelativeSceneNode.setDistanceToDisableTracking(subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking());
         }

         if (localParentNode != null) // Parent of root node is null
         {
            MessageTools.toEuclid(subscriptionNode.getSceneNodeMessage().getTransformToWorld(), nodeToWorldTransform);
            modificationQueue.accept(new SceneGraphNodeReplacement(localNode, localParentNode, nodeToWorldTransform));
         }
      }

      for (ROS2SceneGraphSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         SceneNode localChildNode = sceneGraph.getIDToNodeMap().get(subscriptionChildNode.getSceneNodeMessage().getId());
         if (localChildNode == null && !localTreeFrozen) // New node that wasn't in the local tree
         {
            localChildNode = newNodeSupplier.apply(sceneGraph, subscriptionChildNode);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue);
         }
      }
   }

   public void destroy()
   {

   }
}
