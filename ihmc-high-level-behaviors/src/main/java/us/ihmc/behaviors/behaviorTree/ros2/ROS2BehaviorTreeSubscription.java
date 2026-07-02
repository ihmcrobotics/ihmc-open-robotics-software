package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

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
   private final ROS2Node ros2Node;
   private final ROS2Subscription<BehaviorTreeStateMessage> behaviorTreeStateMessageSubscription;
   private final ConcurrentRingBuffer<BehaviorTreeStateMessage> behaviorTreeStateMessageQueue;
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final HashMap<Long, ROS2BehaviorTreeSubscriptionNode> idToSubscriptionNodesMap = new HashMap<>();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();
   private final HashMap<Long, T> idToLocalNodesMap = new HashMap<>();

   public ROS2BehaviorTreeSubscription(BehaviorTree<?, T> behaviorTree, ROS2Node ros2Node)
   {
      this.behaviorTree = behaviorTree;
      this.ros2Node = ros2Node;

      topic = AutonomyAPI.BEHAVIOR_TREE.getTopic(behaviorTree.getCRDTInfo().getActorDesignation().getIncomingQualifier());
      int maxClientSoftLimit = 3; // This buffer prevents race conditions between clients
      behaviorTreeStateMessageQueue = new ConcurrentRingBuffer<>(BehaviorTreeStateMessage::new, maxClientSoftLimit);
      Throttler warningThrottler = new Throttler().setFrequency(1.0);
      MutableInt droppedMessages = new MutableInt(0);
      behaviorTreeStateMessageSubscription = ros2Node.createSubscription(topic, reader ->
      {
         BehaviorTreeStateMessage nextMessage;
         while ((nextMessage = behaviorTreeStateMessageQueue.next()) == null)
         {
            droppedMessages.increment();

            if (warningThrottler.run())
            {
               LogTools.warn("Concurrent ring buffer has been full! Queue size: {} Have dropped {} oldest messages...",
                             maxClientSoftLimit,
                             droppedMessages.intValue());
               droppedMessages.setValue(0);
            }

            behaviorTreeStateMessageQueue.poll();
            behaviorTreeStateMessageQueue.read();
            behaviorTreeStateMessageQueue.flush();
         }

         BehaviorTreeStateMessage readMessage = reader.read();
         if (readMessage != null)
         {
            nextMessage.set(readMessage);

            ++numberOfMessagesReceived;
            for (Runnable messageRecievedCallback : messageRecievedCallbacks)
            {
               messageRecievedCallback.run();
            }

            long receivedSequenceID = readMessage.getSequenceId();
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

            behaviorTreeStateMessageQueue.commit();
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
         double time = Conversions.nanosecondsToSeconds(System.nanoTime() - before);
         double dt = UnitConversions.hertzToSeconds(ROS2BehaviorTree.SYNC_FREQUENCY);
         if (time > dt)
            LogTools.warn(("Node %s:%d took a long time to create: %.2f > dt %.3f. "
                + "It should take significantly less than one tick.").formatted(localNode.getDefinition().getName(), localNode.getState().getID(), time, dt));
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

      idToSubscriptionNodesMap.put((long) subscriptionNode.getBehaviorTreeNodeStateMessage().getId(), subscriptionNode);

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
      messageRecievedCallbacks.clear();
      ros2Node.destroySubscription(behaviorTreeStateMessageSubscription);
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
