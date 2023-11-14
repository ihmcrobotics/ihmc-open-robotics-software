package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;

public class ROS2BehaviorTreeSubscription
{
   private final ROS2Topic<BehaviorTreeStateMessage> topic;
   private final IHMCROS2Input<BehaviorTreeStateMessage> behaviorTreeSubscription;
   private final BehaviorTreeState behaviorTreeState;
   private final Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSetter;
   private long numberOfMessagesReceived = 0;
   private BehaviorTreeStateMessage latestBehaviorTreeMessage;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTreeState,
                                       Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSetter,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.rootNodeSetter = rootNodeSetter;

      topic = AutonomyAPI.BEAVIOR_TREE.getTopic(behaviorTreeState.getCRDTInfo().getActorDesignation().getIncomingQualifier());
      behaviorTreeSubscription = ros2PublishSubscribeAPI.subscribe(topic);
   }

   public void update()
   {
      if (behaviorTreeSubscription.getMessageNotification().poll())
      {
         ++numberOfMessagesReceived;
         latestBehaviorTreeMessage = behaviorTreeSubscription.getMessageNotification().read();

         subscriptionRootNode.clear();
         subscriptionNodeDepthFirstIndex.setValue(0);
         boolean subscriptionRootIsNull = latestBehaviorTreeMessage.getBehaviorTreeTypes().isEmpty();
         if (!subscriptionRootIsNull)
            buildSubscriptionTree(latestBehaviorTreeMessage, subscriptionRootNode);

         behaviorTreeState.fromMessage(latestBehaviorTreeMessage);

         if (!behaviorTreeState.isFrozen())
         {
            // Clears all unfrozen nodes
            behaviorTreeState.modifyTree(modificationQueue ->
                                   modificationQueue.queueModification(behaviorTreeState.getTreeRebuilder().getClearSubtreeModification()));
         }

         behaviorTreeState.modifyTree(modificationQueue ->
         {
            // When the root node is swapped out, we freeze the reference to the new one
            boolean treeRootReferenceFrozen = behaviorTreeState.isFrozen();

            boolean allowReplicatingRoot = !treeRootReferenceFrozen;
            BehaviorTreeNodeExtension<?, ?, ?, ?> rootNode
                  = subscriptionRootIsNull ? null : retrieveOrReplicateLocalNode(subscriptionRootNode, allowReplicatingRoot);

            if (rootNode != null)
            {
               // The root node's parent is "null"
               updateLocalTreeFromSubscription(subscriptionRootNode, rootNode, null, modificationQueue, treeRootReferenceFrozen);
            }
            else if (!treeRootReferenceFrozen)
            {
               rootNodeSetter.accept(null);
            }

            modificationQueue.queueModification(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversModification());
         });
      }
   }

   private BehaviorTreeNodeExtension<?, ?, ?, ?> retrieveOrReplicateLocalNode(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean allowReplication)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      BehaviorTreeNodeExtension<?, ?, ?, ?> localNode = behaviorTreeState.getTreeRebuilder().getReplacementNode(nodeID);
      if (localNode == null && allowReplication) // New node that wasn't in the local tree; duplicate of one on the other side
      {
         Class<?> nodeTypeClass = BehaviorTreeDefinitionRegistry.getNodeStateClass(subscriptionNode.getType());
         LogTools.info("Replicating node: {}:{} Actor: {}",
                       subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getDescription(),
                       nodeID,
                       behaviorTreeState.getCRDTInfo().getActorDesignation().name());
         localNode = behaviorTreeState.getNodeStateBuilder().createNode(nodeTypeClass, nodeID, behaviorTreeState.getCRDTInfo());
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
            modificationQueue.queueSetRootNode(localNode, rootNodeSetter);
         else
            modificationQueue.queueAddNode(localNode, localParentNode);
      }

      for (int i = 0; i < subscriptionNode.getChildren().size(); i++)
      {
         anAncestorIsFrozen |= localNode.getState().isFrozen();

         BehaviorTreeNodeExtension<?, ?, ?, ?> localChildNode;
         if (anAncestorIsFrozen)
         {
            localChildNode = (BehaviorTreeNodeExtension<?, ?, ?, ?>) localNode.getChildren().get(i);
         }
         else
         {
            localChildNode = retrieveOrReplicateLocalNode(subscriptionNode.getChildren().get(i), true);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionNode.getChildren().get(i), localChildNode, localNode, modificationQueue, anAncestorIsFrozen);
         }
      }

      // Update the state after iterating over children, because node can unfreeze at this time
      // Each state handles which fields it updates based on its frozen status
      ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());
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
