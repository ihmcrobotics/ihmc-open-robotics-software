package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.UnitConversions;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Handles the ROS 2 tree subscription which handles the CRDT sychronization with other UIs and the robot.
 */
public class ROS2BehaviorTreeSubscription<T extends BehaviorTreeNode<T, ?, ?>>
{
   private final ROS2Topic<BehaviorTreeStateMessage> topic;
   private final ArrayList<Runnable> messageRecievedCallbacks = new ArrayList<>();
   private final BehaviorTree<?, T> behaviorTree;
   private long numberOfMessagesReceived = 0;
   private long previousSequenceID = -1;
   private long messageDropCount = 0;
   private long outOfOrderCount = 0;
   private long sequenceId;
   private int numberOfOnRobotNodes = 0;
   private final ConcurrentRingBuffer<BehaviorTreeStateMessage> behaviorTreeStateMessageQueue;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final HashMap<Long, ROS2BehaviorTreeSubscriptionNode> idToSubscriptionNodesMap = new HashMap<>();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();
   private final HashMap<Long, T> idToLocalNodesMap = new HashMap<>();

   public ROS2BehaviorTreeSubscription(BehaviorTree<?, T> behaviorTree, ROS2Helper ros2PublishSubscribeAPI)
   {
      this.behaviorTree = behaviorTree;

      topic = AutonomyAPI.BEHAVIOR_TREE.getTopic(behaviorTree.getCRDTInfo().getActorDesignation().getIncomingQualifier());
      int maxClientSoftLimit = 3; // This buffer prevents race conditions between clients
      behaviorTreeStateMessageQueue = ros2PublishSubscribeAPI.subscribeViaQueue(topic, maxClientSoftLimit, behaviorTreeStateMessage ->
      {
         ++numberOfMessagesReceived;
         for (Runnable messageRecievedCallback : messageRecievedCallbacks)
         {
            messageRecievedCallback.run();
         }

         if (behaviorTreeStateMessage != null)
         {
            long receivedSequenceID = behaviorTreeStateMessage.getSequenceId();
            long expectedSequenceID = previousSequenceID + 1;
            long difference = receivedSequenceID - expectedSequenceID;

            if (Math.abs(difference) < ROS2BehaviorTree.SYNC_FREQUENCY) // Account for restarts
            {
               if (difference > 0)
               {
                  messageDropCount += difference;
               }
               else if (difference < 0)
               {
                  outOfOrderCount -= difference;
               }
            }

            previousSequenceID = receivedSequenceID;
         }
      });
   }

   public void update()
   {
      while (behaviorTreeStateMessageQueue.poll())
      {
         BehaviorTreeStateMessage behaviorTreeStateMessage = behaviorTreeStateMessageQueue.read();

         sequenceId = behaviorTreeStateMessage.getSequenceId();
         numberOfOnRobotNodes = behaviorTreeStateMessage.getBehaviorTreeIndices().size();

         subscriptionRootNode.clear();
         subscriptionNodeDepthFirstIndex.setValue(0);
         idToSubscriptionNodesMap.clear();
         boolean subscriptionRootIsNull = behaviorTreeStateMessage.getBehaviorTreeTypes().isEmpty();
         if (!subscriptionRootIsNull)
            buildSubscriptionTree(behaviorTreeStateMessage, subscriptionRootNode);

         behaviorTree.fromMessage(behaviorTreeStateMessage);

         // The algorithm to support added, removed, and moved nodes:
         // 1. Map the nodes by ID
         // 2. As we traverse the tree, remove the IDs from the map
         //    2a. If children modified, traverse the message's children
         //    2b. Else, traverse our local children list
         // 3. Any nodes remaining in the map afterwards get destroyed
         idToLocalNodesMap.clear();
         if (behaviorTree.getRootNode() != null)
            BehaviorTreeTools.runForSubtreeNodes((T) behaviorTree.getRootNode(),
                                                 node -> idToLocalNodesMap.put(node.getState().getID(), node));

         behaviorTree.modifyTreeTopology(topologyOperationQueue ->
         {
            BehaviorTreeRootNode<T> rootNode = behaviorTree.getRootNode();

            boolean rootReferenceModificationIncoming = behaviorTree.getRootReferenceModification().isModificationIncoming();
            if (rootReferenceModificationIncoming)
            {
               rootNode = subscriptionRootIsNull ? null :
                                (BehaviorTreeRootNode<T>) retrieveOrReplicateLocalNode(subscriptionRootNode, rootReferenceModificationIncoming, null);
               topologyOperationQueue.queueSetRootNode(rootNode);
            }

            if (rootNode != null)
               idToLocalNodesMap.remove(rootNode.getState().getID());

            // We need to traverse the tree even if there are no remote nodes to match,
            // because we might have the most up to date version
            if (rootNode != null)
               retrieveOrReplicateSubreeFromSubscription(subscriptionRootNode, (T) rootNode, rootNode, topologyOperationQueue);

            // These nodes were removed from the tree
            for (T value : idToLocalNodesMap.values())
               value.destroy();
            idToLocalNodesMap.clear();
         });

         behaviorTreeStateMessageQueue.flush();
      }
   }

   private void retrieveOrReplicateSubreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                          T localNode,
                                                          BehaviorTreeRootNode<T> rootNode,
                                                          BehaviorTreeTopologyOperationQueue<T> topologyOperationQueue)
   {
      boolean existsMatchingSubscriptionNode = subscriptionNode != null && subscriptionNode.getDefinitionClass() != null;
      if (existsMatchingSubscriptionNode && sequenceId != subscriptionNode.getSequenceId())
      {
         LogTools.error(("Sequence ID %d != subscriptionNode.sequenceId %d. We took too long in this update and the queue wrapped around.")
                              .formatted(sequenceId, subscriptionNode.getSequenceId()));
      }
      if (existsMatchingSubscriptionNode && subscriptionNode.getBehaviorTreeNodeStateMessage().getId() != localNode.getState().getID())
      {
         LogTools.error("ID mismatch: Subscription node: %s:%d  Local node: %s:%d  Skipping update."
                              .formatted(subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getName(),
                                         subscriptionNode.getBehaviorTreeNodeStateMessage().getId(),
                                         localNode.getDefinition().getName(),
                                         localNode.getState().getID()));
         existsMatchingSubscriptionNode = false;
      }

      // Update the node first, to detect incoming modifications
      if (existsMatchingSubscriptionNode)
      {
         if (subscriptionNode.getPackedType() == BehaviorTreeStateMessage.PARTIAL_DATA)
         {
            localNode.getDefinition().fromMessage(subscriptionNode.getBehaviorTreeNodeDefinitionMessage(), true);
            localNode.getState().fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());

            if (localNode.getDefinition().isModificationIncoming())
            {
               // This is caused by dropped messages or newly online processes
               LogTools.debug(() -> "%s: Partial data is newer than what we have. %s"
                     .formatted(behaviorTree.getCRDTInfo().getActorDesignation(),
                                localNode.getDefinition().getName()));
               // We need to ask to get sent the full data
               localNode.getDefinition().requestSendFullData();
            }
         }
         else
         {
            localNode.getDefinition().confirmReceivedFullData();
            ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());
         }
      }

      // Traverse the latest children list
      if (existsMatchingSubscriptionNode && localNode.getDefinition().getChildrenModification().isModificationIncoming())
      {
         topologyOperationQueue.queueClearImmediateChildren(localNode);
         for (ROS2BehaviorTreeSubscriptionNode subscriptionChild : subscriptionNode.getChildren())
         {
            T localChildNode = retrieveOrReplicateLocalNode(subscriptionChild, true, rootNode);
            if (localChildNode != null)
            {
               topologyOperationQueue.queueAppendChild(localNode, localChildNode);
               retrieveOrReplicateSubreeFromSubscription(subscriptionChild, localChildNode, rootNode, topologyOperationQueue);
               idToLocalNodesMap.remove(localChildNode.getState().getID());
            }
         }
      }
      else
      {
         for (T localChildNode : localNode.getChildren())
         {
            ROS2BehaviorTreeSubscriptionNode subscriptionChild = idToSubscriptionNodesMap.get(localChildNode.getState().getID());
            retrieveOrReplicateSubreeFromSubscription(subscriptionChild, localChildNode, rootNode, topologyOperationQueue);
            idToLocalNodesMap.remove(localChildNode.getState().getID());
         }
      }
   }

   private T retrieveOrReplicateLocalNode(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean allowReplication, BehaviorTreeRootNode<T> rootNode)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      T localNode = idToLocalNodesMap.get(nodeID);
      if (localNode == null && allowReplication) // New node that wasn't in the local tree; duplicate of one on the other side
      {
         LogTools.info("%s: Seq # %d Replicating node: %s:%d Packed as: %s Definition: %s Actor: %s".formatted(
               behaviorTree.getCRDTInfo().getActorDesignation().name(),
               subscriptionNode.getSequenceId(),
               subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getName(),
               nodeID,
               subscriptionNode.getPackedType(),
               subscriptionNode.getDefinitionClass().getSimpleName(),
               behaviorTree.getCRDTInfo().getActorDesignation().name()));
         long before = System.nanoTime();
         if (rootNode == null)
            localNode = (T) behaviorTree.getNodeBuilder().createRootNode(nodeID);
         else
            localNode = behaviorTree.getNodeBuilder().createNode(subscriptionNode.getDefinitionClass(),
                                                                 nodeID,
                                                                 rootNode);
         if (Conversions.nanosecondsToSeconds(System.nanoTime() - before) > UnitConversions.hertzToSeconds(ROS2BehaviorTree.SYNC_FREQUENCY))
            LogTools.warn(("Node took a long time to create. %s:%d"
                + "It should take significantly less than one tick.").formatted(localNode.getDefinition().getName(), localNode.getState().getID()));
         if (subscriptionNode.getPackedType() == BehaviorTreeStateMessage.PARTIAL_DATA)
         {
            LogTools.debug("Cannot replicate node fully from partial data!");
            localNode.getDefinition().requestSendFullData();
         }
      }

      return localNode;
   }

   /** Build an intermediate tree representation of the message, which helps to sync with the actual tree. */
   private void buildSubscriptionTree(BehaviorTreeStateMessage behaviorTreeStateMessage, ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      subscriptionNode.setSequenceId(behaviorTreeStateMessage.getSequenceId());
      byte nodeType = behaviorTreeStateMessage.getBehaviorTreeTypes().get(subscriptionNodeDepthFirstIndex.intValue());
      int indexInTypesList = (int) behaviorTreeStateMessage.getBehaviorTreeIndices().get(subscriptionNodeDepthFirstIndex.intValue());
      subscriptionNode.setPackedType(nodeType);

      ROS2BehaviorTreeMessageTools.packSubscriptionNode(nodeType, indexInTypesList, behaviorTreeStateMessage, subscriptionNode);

      idToSubscriptionNodesMap.put(subscriptionNode.getBehaviorTreeNodeStateMessage().getId(), subscriptionNode);

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

   public long getPreviousSequenceID()
   {
      return previousSequenceID;
   }

   public long getMessageDropCount()
   {
      return messageDropCount - outOfOrderCount;
   }

   public long getOutOfOrderCount()
   {
      return outOfOrderCount;
   }

   public int getNumberOfOnRobotNodes()
   {
      return numberOfOnRobotNodes;
   }
}
