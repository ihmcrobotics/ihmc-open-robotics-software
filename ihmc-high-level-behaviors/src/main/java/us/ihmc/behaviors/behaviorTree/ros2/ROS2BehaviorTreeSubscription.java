package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeAdd;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeSetRoot;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

import java.util.function.Consumer;

public class ROS2BehaviorTreeSubscription
{
   private final IHMCROS2Input<BehaviorTreeStateMessage> behaviorTreeSubscription;
   private final BehaviorTreeState behaviorTreeState;
   private final Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSetter;
   private long numberOfMessagesReceived = 0;
   private BehaviorTreeStateMessage latestBehaviorTreeMessage;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   /**
    * @param ioQualifier If in the on-robot autonomy process, COMMAND, else STATUS
    */
   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTreeState,
                                       Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSetter,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                       ROS2IOTopicQualifier ioQualifier)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.rootNodeSetter = rootNodeSetter;

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

         if (!behaviorTreeState.getLocalTreeFrozen() && behaviorTreeState.getRootNode() != null)
         {
            // First clear the tree, storing all nodes by ID in the map in the tree rebuilder
            behaviorTreeState.modifyTree(modificationQueue ->
                                               modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getClearSubtreeModification()));
         }

         behaviorTreeState.modifyTree(modificationQueue ->
         {
            BehaviorTreeNodeExtension<?, ?, ?, ?> rootNode = recallNodeByIDOrCreate(subscriptionRootNode);

            updateLocalTreeFromSubscription(subscriptionRootNode, rootNode, null, modificationQueue);

            if (!behaviorTreeState.getLocalTreeFrozen())
            {
               modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversModification());
            }
         });
      }
   }

   private BehaviorTreeNodeExtension<?, ?, ?, ?> recallNodeByIDOrCreate(ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      BehaviorTreeNodeExtension<?, ?, ?, ?> localNode = behaviorTreeState.getTreeRebuilder().getReplacementNode(nodeID);
      if (localNode == null && !behaviorTreeState.getLocalTreeFrozen()) // New node that wasn't in the local tree
      {
         Class<?> nodeTypeClass = BehaviorTreeDefinitionRegistry.getNodeStateClass(subscriptionNode.getType());
         localNode = behaviorTreeState.getNodeStateBuilder().createNode(nodeTypeClass, nodeID);
      }

      return localNode;
   }

   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                BehaviorTreeNodeExtension<?, ?, ?, ?> localNode,
                                                BehaviorTreeNodeExtension<?, ?, ?, ?> localParentNode,
                                                BehaviorTreeModificationQueue modificationQueue)
   {
      // If the node was recently modified by the operator, the node does not accept
      // updates of these values. This is to allow the operator's changes to propagate
      // and so it doesn't get overriden immediately by an out of date message coming from the robot.
      // On the robot side, this will always get updated because there is no operator.
      if (!behaviorTreeState.getLocalTreeFrozen())
      {
         if (localParentNode == null)
            modificationQueue.accept(new BehaviorTreeNodeSetRoot(localNode, rootNodeSetter));
         else
            modificationQueue.accept(new BehaviorTreeNodeAdd(localNode, localParentNode));

         ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());
      }

      for (ROS2BehaviorTreeSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         BehaviorTreeNodeExtension<?, ?, ?, ?> localChildNode = recallNodeByIDOrCreate(subscriptionChildNode);

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue);
         }
      }
   }

   /** Build an intermediate tree representation of the message, which helps to sync with the actual tree. */
   private void buildSubscriptionTree(BehaviorTreeStateMessage behaviorTreeStateMessage, ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      byte nodeType = behaviorTreeStateMessage.getBehaviorTreeTypes().get(subscriptionNodeDepthFirstIndex.intValue());
      int indexInTypesList = (int) behaviorTreeStateMessage.getBehaviorTreeIndices().get(subscriptionNodeDepthFirstIndex.intValue());
      subscriptionNode.setType(nodeType);

      ROS2BehaviorTreeMessageTools.packSubscriptionNode(nodeType, indexInTypesList, behaviorTreeStateMessage, subscriptionNode);

      for (int i = 0; i < subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getNumberOfChildren(); i++)
      {
         ROS2BehaviorTreeSubscriptionNode subscriptionTreeChildNode = new ROS2BehaviorTreeSubscriptionNode();
         subscriptionNodeDepthFirstIndex.increment();
         buildSubscriptionTree(behaviorTreeStateMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
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
