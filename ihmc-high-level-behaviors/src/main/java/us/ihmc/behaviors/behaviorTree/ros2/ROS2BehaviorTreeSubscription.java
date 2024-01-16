package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.function.Consumer;

@SuppressWarnings({"unchecked"}) // Sometimes in this class we need to be a little unsafe
public class ROS2BehaviorTreeSubscription<T extends BehaviorTreeNodeLayer<T, ?, ?, ?>>
{
   private final ROS2Topic<BehaviorTreeStateMessage> topic;
   private final Notification recievedMessageNotification = new Notification();
   private final ArrayList<Runnable> messageRecievedCallbacks = new ArrayList<>();
   private final BehaviorTreeState behaviorTreeState;
   private final Consumer<T> rootNodeSetter;
   private long numberOfMessagesReceived = 0;
   private BehaviorTreeStateMessage behaviorTreeStateMessage = new BehaviorTreeStateMessage();
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTreeState,
                                       Consumer<T> rootNodeSetter,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.rootNodeSetter = rootNodeSetter;

      topic = AutonomyAPI.BEAVIOR_TREE.getTopic(behaviorTreeState.getCRDTInfo().getActorDesignation().getIncomingQualifier());
      ros2PublishSubscribeAPI.subscribeViaCallback(topic, recievedMessageNotification, behaviorTreeStateMessage);
   }

   public void update()
   {
      if (recievedMessageNotification.poll())
      {
         ++numberOfMessagesReceived;
         for (Runnable messageRecievedCallback : messageRecievedCallbacks)
         {
            messageRecievedCallback.run();
         }

         subscriptionRootNode.clear();
         subscriptionNodeDepthFirstIndex.setValue(0);
         boolean subscriptionRootIsNull = behaviorTreeStateMessage.getBehaviorTreeTypes().isEmpty();
         if (!subscriptionRootIsNull)
            buildSubscriptionTree(behaviorTreeStateMessage, subscriptionRootNode);

         behaviorTreeState.fromMessage(behaviorTreeStateMessage);

         if (!behaviorTreeState.isFrozen())
         {
            // Clears all unfrozen nodes
            behaviorTreeState.modifyTreeTopology(topologyOperationQueue ->
                                   topologyOperationQueue.queueOperation(behaviorTreeState.getTreeRebuilder().getClearSubtreeOperation()));
         }

         behaviorTreeState.modifyTreeTopology(topologyOperationQueue ->
         {
            // When the root node is swapped out, we freeze the reference to the new one
            boolean treeRootReferenceFrozen = behaviorTreeState.isFrozen();

            boolean allowReplicatingRoot = !treeRootReferenceFrozen;
            T rootNode
                  = subscriptionRootIsNull ? null : retrieveOrReplicateLocalNode(subscriptionRootNode, allowReplicatingRoot);

            if (rootNode != null)
            {
               // The root node's parent is "null"
               updateLocalTreeFromSubscription(subscriptionRootNode, rootNode, null, topologyOperationQueue, treeRootReferenceFrozen);
            }
            else if (!treeRootReferenceFrozen)
            {
               rootNodeSetter.accept(null);
            }

            topologyOperationQueue.queueOperation(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversOperation());
         });
      }
   }

   private T retrieveOrReplicateLocalNode(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean allowReplication)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      T localNode = (T) behaviorTreeState.getTreeRebuilder().getReplacementNode(nodeID);
      if (localNode == null && allowReplication) // New node that wasn't in the local tree; duplicate of one on the other side
      {
         Class<?> nodeTypeClass = BehaviorTreeDefinitionRegistry.getNodeStateClass(subscriptionNode.getType());
         LogTools.info("Replicating node: {}:{} Actor: {}",
                       subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getDescription(),
                       nodeID,
                       behaviorTreeState.getCRDTInfo().getActorDesignation().name());
         localNode = (T) behaviorTreeState.getNodeStateBuilder()
                                          .createNode(nodeTypeClass, nodeID, behaviorTreeState.getCRDTInfo(), behaviorTreeState.getSaveFileDirectory());
      }

      return localNode;
   }

   private void updateLocalTreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                T localNode,
                                                T localParentNode,
                                                BehaviorTreeTopologyOperationQueue topologyOperationQueue,
                                                boolean anAncestorIsFrozen)
   {
      // We just add nodes if they would not be part of a frozen subtree.
      if (!anAncestorIsFrozen)
      {
         if (localParentNode == null)
            topologyOperationQueue.queueSetRootNode(localNode, rootNodeSetter);
         else
            topologyOperationQueue.queueAddNode(localNode, localParentNode);
      }

      for (int i = 0; i < subscriptionNode.getChildren().size(); i++)
      {
         anAncestorIsFrozen |= localNode.getState().isFrozen();

         T localChildNode = null;
         if (anAncestorIsFrozen)
         {
            // In the case of locally nodes that just got added or removed, only update children with matching IDs.
            // This'll just be for a few updates.
            for (T possibleMatch : localNode.getChildren())
            {
               if (possibleMatch.getState().getID() == subscriptionNode.getChildren().get(i).getBehaviorTreeNodeStateMessage().getId())
               {
                  localChildNode = possibleMatch;
               }
            }
         }
         else
         {
            localChildNode = (T) retrieveOrReplicateLocalNode(subscriptionNode.getChildren().get(i), true);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionNode.getChildren().get(i), localChildNode, localNode, topologyOperationQueue, anAncestorIsFrozen);
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

   public void registerMessageReceivedCallback(Runnable callback)
   {
      messageRecievedCallbacks.add(callback);
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }

   public BehaviorTreeStateMessage getBehaviorTreeStateMessage()
   {
      return behaviorTreeStateMessage;
   }
}
