package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeRebuildSubtree;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreeSubscription
{
   private final IHMCROS2Input<BehaviorTreeStateMessage> behaviorTreeSubscription;
   private final BehaviorTreeState behaviorTree;
   private long numberOfMessagesReceived = 0;
   private boolean localTreeFrozen = false;
   private BehaviorTreeStateMessage latestBehaviorTreeMessage;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   /**
    * @param ioQualifier If in the on-robot autonomy process, COMMAND, else STATUS
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
      if (behaviorTreeSubscription.getMessageNotification().poll())
      {
         ++numberOfMessagesReceived;
         latestBehaviorTreeMessage = behaviorTreeSubscription.getMessageNotification().read();

         subscriptionRootNode.clear();
         subscriptionNodeDepthFirstIndex.setValue(0);
         buildSubscriptionTree(latestBehaviorTreeMessage, subscriptionRootNode);

         // If the tree was recently modified by the operator, we do not accept
         // updates the structure of the tree.
         localTreeFrozen = false;
         checkTreeModified(behaviorTree.getRootNode());

         if (!localTreeFrozen)
            behaviorTree.getNextID().setValue(latestBehaviorTreeMessage.getNextId());

         behaviorTree.modifyTree(modificationQueue ->
         {
            BehaviorTreeRebuildSubtree rebuildSubtree = null;
            if (!localTreeFrozen)
            {
               rebuildSubtree = new BehaviorTreeRebuildSubtree(behaviorTree.getRootNode());
               modificationQueue.accept(rebuildSubtree.getClearSubtreeModification());
            }

            updateLocalTreeFromSubscription(subscriptionRootNode, behaviorTree.getRootNode(), null, modificationQueue);

            if (rebuildSubtree != null)
               modificationQueue.accept(rebuildSubtree.getDestroyLeftoversModification());
         });
      }
   }


   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                BehaviorTreeNodeState localNode,
                                                BehaviorTreeNodeState localParentNode,
                                                BehaviorTreeModificationQueue modificationQueue)
   {
      // Set fields only modifiable by the robot
      localNode.setIsActive(subscriptionNode.getBehaviorTreeNodeStateMessage().getIsActive());

      // If the node was recently modified by the operator, the node does not accept
      // updates of these values. This is to allow the operator's changes to propagate
      // and so it doesn't get overriden immediately by an out of date message coming from the robot.
      // On the robot side, this will always get updated because there is no operator.
      if (!localTreeFrozen)
      {

      }

      for (ROS2BehaviorTreeSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         BehaviorTreeNodeState localChildNode = behaviorTree.getIDToNodeMap().get(subscriptionChildNode.getBehaviorTreeNodeStateMessage().getId());
         if (localChildNode == null && !localTreeFrozen) // New node that wasn't in the local tree
         {
            localChildNode = behaviorTree.getNewNodeSupplier().apply(ROS2BehaviorTreeTools.getNodeStateClass(subscriptionChildNode.getType()));
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue);
         }
      }
   }

   private void checkTreeModified(BehaviorTreeNodeState localNode)
   {
      localTreeFrozen |= localNode.isFrozenFromModification();

      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         checkTreeModified(child);
      }
   }

   /** Build an intermediate tree representation of the message, which helps to sync with the actual tree. */
   private void buildSubscriptionTree(BehaviorTreeStateMessage behaviorTreeStateMessage, ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {

   }

   public void destroy()
   {

   }

   public IHMCROS2Input<BehaviorTreeStateMessage> getBehaviorTreeSubscription()
   {
      return behaviorTreeSubscription;
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }

   public BehaviorTreeStateMessage getLatestBehaviorTreeMessage()
   {
      return latestBehaviorTreeMessage;
   }

   public boolean getLocalTreeFrozen()
   {
      return localTreeFrozen;
   }
}
