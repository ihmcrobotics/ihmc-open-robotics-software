package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModificationQueue;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreeSubscription
{
   private final IHMCROS2Input<BehaviorTreeStateMessage> behaviorTreeSubscription;
   private final BehaviorTreeState behaviorTreeState;
   private long numberOfMessagesReceived = 0;
   private BehaviorTreeStateMessage latestBehaviorTreeMessage;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   /**
    * @param ioQualifier If in the on-robot autonomy process, COMMAND, else STATUS
    */
   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTreeState,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                       ROS2IOTopicQualifier ioQualifier)
   {
      this.behaviorTreeState = behaviorTreeState;

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
         behaviorTreeState.checkTreeModified();

         if (!behaviorTreeState.getLocalTreeFrozen())
            behaviorTreeState.getNextID().setValue(latestBehaviorTreeMessage.getNextId());

         behaviorTreeState.modifyTree(modificationQueue ->
         {
            if (!behaviorTreeState.getLocalTreeFrozen())
            {
               modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getClearSubtreeModification());
            }

            updateLocalTreeFromSubscription(subscriptionRootNode, behaviorTreeState.getRootNode(), null, modificationQueue);

            if (!behaviorTreeState.getLocalTreeFrozen())
            {
               modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversModification());
            }
         });
      }
   }

   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                BehaviorTreeNodeStateSupplier localNode,
                                                BehaviorTreeNodeStateSupplier localParentNode,
                                                BehaviorTreeStateModificationQueue modificationQueue)
   {
      // Set fields only modifiable by the robot
      localNode.getState().setIsActive(subscriptionNode.getBehaviorTreeNodeStateMessage().getIsActive());

      // If the node was recently modified by the operator, the node does not accept
      // updates of these values. This is to allow the operator's changes to propagate
      // and so it doesn't get overriden immediately by an out of date message coming from the robot.
      // On the robot side, this will always get updated because there is no operator.
      if (!behaviorTreeState.getLocalTreeFrozen())
      {
         modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getReplacementModification(localNode.getState().getID(), localParentNode));
      }

      for (ROS2BehaviorTreeSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         long childNodeID = subscriptionChildNode.getBehaviorTreeNodeStateMessage().getId();
         BehaviorTreeNodeStateSupplier localChildNode = behaviorTreeState.getTreeRebuilder().getReplacementNode(childNodeID);
         if (localChildNode == null && !behaviorTreeState.getLocalTreeFrozen()) // New node that wasn't in the local tree
         {
            Class<?> nodeTypeClass = BehaviorTreeDefinitionRegistry.getNodeStateClass(subscriptionChildNode.getType());
            localChildNode = behaviorTreeState.getNodeStateBuilder().createNode(nodeTypeClass, childNodeID);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue);
         }
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
}
