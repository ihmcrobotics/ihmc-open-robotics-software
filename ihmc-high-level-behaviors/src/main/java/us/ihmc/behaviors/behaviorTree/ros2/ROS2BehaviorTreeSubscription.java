package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeExtensionAdd;
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
         behaviorTreeState.countFrozenNodes();

         behaviorTreeState.fromMessage(latestBehaviorTreeMessage);

         if (behaviorTreeState.getRootNode() != null)
         {
            // First clear the tree, storing all nodes by ID in the map in the tree rebuilder
            behaviorTreeState.modifyTree(modificationQueue ->
                                               modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getClearSubtreeModification()));
         }

         behaviorTreeState.modifyTree(modificationQueue ->
         {
            // When the root node is swapped out, we freeze the reference to the new one
            boolean treeRootReferenceFrozen = behaviorTreeState.isFrozenFromModification();

            BehaviorTreeNodeExtension<?, ?, ?, ?> rootNode = recallNodeByIDOrCreate(subscriptionRootNode, treeRootReferenceFrozen);

            // The root node's parent is "null"
            updateLocalTreeFromSubscription(subscriptionRootNode, rootNode, null, modificationQueue, treeRootReferenceFrozen);

            modificationQueue.accept(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversModification());
         });
      }
   }

   private BehaviorTreeNodeExtension<?, ?, ?, ?> recallNodeByIDOrCreate(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean anAncestorIsFrozen)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      BehaviorTreeNodeExtension<?, ?, ?, ?> localNode = behaviorTreeState.getTreeRebuilder().getReplacementNode(nodeID);
      if (localNode == null && !anAncestorIsFrozen) // New node that wasn't in the local tree
      {
         Class<?> nodeTypeClass = BehaviorTreeDefinitionRegistry.getNodeStateClass(subscriptionNode.getType());
         localNode = behaviorTreeState.getNodeStateBuilder().createNode(nodeTypeClass, nodeID);
         // Set basic fields just to help in debugging and possibly reducing bugs
         localNode.getDefinition().fromMessage(subscriptionNode.getBehaviorTreeNodeDefinitionMessage());
         localNode.getState().fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());
      }

      return localNode;
   }

   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                BehaviorTreeNodeExtension<?, ?, ?, ?> localNode,
                                                BehaviorTreeNodeExtension<?, ?, ?, ?> localParentNode,
                                                BehaviorTreeModificationQueue modificationQueue,
                                                boolean anAncestorIsFrozen)
   {
      // We just add nodes if they would not be part of a frozen subtree.
      if (!anAncestorIsFrozen)
      {
         if (localParentNode == null)
            modificationQueue.accept(new BehaviorTreeNodeSetRoot(localNode, rootNodeSetter));
         else
            modificationQueue.accept(new BehaviorTreeNodeExtensionAdd(localNode, localParentNode));
      }

      // Each state handles which fields it updates based on its frozen status
      ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());

      for (ROS2BehaviorTreeSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         anAncestorIsFrozen |= localNode.getState().isFrozenFromModification();

         BehaviorTreeNodeExtension<?, ?, ?, ?> localChildNode = recallNodeByIDOrCreate(subscriptionChildNode, anAncestorIsFrozen);

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue, anAncestorIsFrozen);
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
