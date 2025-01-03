package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

/**
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 */
public class ROS2BehaviorTreeSubscription<HLT extends BehaviorTreeNodeHighLayer<HLT, ? ,?>>
{
   private final ROS2Topic<BehaviorTreeStateMessage> topic;
   private final ArrayList<Runnable> messageRecievedCallbacks = new ArrayList<>();
   private final BehaviorTree<HLT> behaviorTree;
   private final Consumer<HLT> rootNodeSetter;
   private long numberOfMessagesReceived = 0;
   private long previousSequenceID = -1;
   private long messageDropCount = 0;
   private long outOfOrderCount = 0;
   private int numberOfOnRobotNodes = 0;
   private final SwapReference<BehaviorTreeStateMessage> behaviorTreeStateMessageSwapReference;
   private final Notification recievedMessageNotification = new Notification();
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final HashMap<Long, ROS2BehaviorTreeSubscriptionNode> idToSubscriptionNodesMap = new HashMap<>();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();
   private final HashMap<Long, HLT> idToLocalNodesMap = new HashMap<>();

   public ROS2BehaviorTreeSubscription(BehaviorTree<HLT> behaviorTree,
                                       Consumer<HLT> rootNodeSetter,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTree = behaviorTree;
      this.rootNodeSetter = rootNodeSetter;

      topic = AutonomyAPI.BEHAVIOR_TREE.getTopic(behaviorTree.getCRDTInfo().getActorDesignation().getIncomingQualifier());
      behaviorTreeStateMessageSwapReference = ros2PublishSubscribeAPI.subscribeViaSwapReference(topic, behaviorTreeStateMessage ->
      {
         ++numberOfMessagesReceived;
         for (Runnable messageRecievedCallback : messageRecievedCallbacks)
         {
            messageRecievedCallback.run();
         }

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

         recievedMessageNotification.set();
      });
   }

   public void update()
   {
      if (recievedMessageNotification.poll())
      {
         synchronized (behaviorTreeStateMessageSwapReference)
         {
            BehaviorTreeStateMessage behaviorTreeStateMessage = behaviorTreeStateMessageSwapReference.getForThreadTwo();

            numberOfOnRobotNodes = behaviorTreeStateMessage.getBehaviorTreeIndices().size();

            subscriptionRootNode.clear();
            subscriptionNodeDepthFirstIndex.setValue(0);
            idToSubscriptionNodesMap.clear();
            boolean subscriptionRootIsNull = behaviorTreeStateMessage.getBehaviorTreeTypes().isEmpty();
            if (!subscriptionRootIsNull)
               buildSubscriptionTree(behaviorTreeStateMessage, subscriptionRootNode);

            behaviorTree.fromMessage(behaviorTreeStateMessage);

            idToLocalNodesMap.clear();
            if (behaviorTree.getRootNode() != null)
               BehaviorTreeTools.runForSubtreeNodes(behaviorTree.getRootNode(),
                                                    node -> idToLocalNodesMap.put(node.getState().getID(), node));

            behaviorTree.modifyTreeTopology(topologyOperationQueue ->
            {
               HLT rootNode = behaviorTree.getRootNode();

               boolean rootReferenceModificationIncoming = behaviorTree.getRootReferenceModification().isModificationIncoming();
               if (rootReferenceModificationIncoming)
               {
                  rootNode = subscriptionRootIsNull ? null : retrieveOrReplicateLocalNode(subscriptionRootNode, rootReferenceModificationIncoming);
                  topologyOperationQueue.queueSetRootNode(rootNode);
               }

               if (rootNode != null)
                  idToLocalNodesMap.remove(rootNode.getState().getID());

               // We need to traverse the tree even if there are no remote nodes to match,
               // because we might have the most up to date version
               if (rootNode != null)
                  retrieveOrReplicateSubreeFromSubscription(subscriptionRootNode, rootNode, topologyOperationQueue);

               // These nodes were removed from the tree
               for (HLT value : idToLocalNodesMap.values())
                  value.destroy();
               idToLocalNodesMap.clear();
            });
         }
      }
   }

   private void retrieveOrReplicateSubreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                          HLT localNode,
                                                          BehaviorTreeTopologyOperationQueue<HLT> topologyOperationQueue)
   {
      boolean existsMatchingSubscriptionNode = subscriptionNode != null && subscriptionNode.getDefinitionClass() != null;

      // Update the node first, to detect incoming modifications
      if (existsMatchingSubscriptionNode)
      {
         if (subscriptionNode.getPackedType() == BehaviorTreeStateMessage.PARTIAL_DATA)
         {
            localNode.getDefinition().fromMessage(subscriptionNode.getBehaviorTreeNodeDefinitionMessage());
            localNode.getState().fromMessage(subscriptionNode.getBehaviorTreeNodeStateMessage());

            if (localNode.getDefinition().isModificationIncoming())
            {
               // This is caused by dropped messages or newly online processes
               LogTools.error(() -> "%s: Partial data is newer than what we have. %s"
                     .formatted(behaviorTree.getCRDTInfo().getActorDesignation(),
                                localNode.getDefinition().getName()));
               // We need to ask to get sent the full data
               localNode.getDefinition().requestSendFullData();
            }
         }
         else
         {
            ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());
         }
      }

      // Traverse the latest children list
      if (existsMatchingSubscriptionNode && localNode.getDefinition().getChildrenModification().isModificationIncoming())
      {
         topologyOperationQueue.queueClearImmediateChildren(localNode);
         for (ROS2BehaviorTreeSubscriptionNode subscriptionChild : subscriptionNode.getChildren())
         {
            HLT localChildNode = retrieveOrReplicateLocalNode(subscriptionChild, true);
            if (localChildNode != null)
            {
               topologyOperationQueue.queueAppendChild(localNode, localChildNode);
               retrieveOrReplicateSubreeFromSubscription(subscriptionChild, localChildNode, topologyOperationQueue);
               idToLocalNodesMap.remove(localChildNode.getState().getID());
            }
         }
      }
      else
      {
         for (HLT localChildNode : localNode.getChildren())
         {
            ROS2BehaviorTreeSubscriptionNode subscriptionChild = idToSubscriptionNodesMap.get(localChildNode.getState().getID());
            retrieveOrReplicateSubreeFromSubscription(subscriptionChild, localChildNode, topologyOperationQueue);
            idToLocalNodesMap.remove(localChildNode.getState().getID());
         }
      }
   }

   private HLT retrieveOrReplicateLocalNode(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean allowReplication)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      HLT localNode = idToLocalNodesMap.get(nodeID);
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
         if (subscriptionNode.getPackedType() == BehaviorTreeStateMessage.PARTIAL_DATA)
         {
            LogTools.error("Cannot replicate node from partial data!");
         }
         else
         {
            localNode = behaviorTree.getNodeBuilder().createNode(subscriptionNode.getDefinitionClass(),
                                                                 nodeID,
                                                                 behaviorTree.getCRDTInfo(),
                                                                 behaviorTree.getSaveFileDirectory());
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
