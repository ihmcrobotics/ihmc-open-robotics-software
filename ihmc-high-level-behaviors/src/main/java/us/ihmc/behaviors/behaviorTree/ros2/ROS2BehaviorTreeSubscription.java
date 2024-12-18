package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.function.Consumer;

@SuppressWarnings({"unchecked"}) // Sometimes in this class we need to be a little unsafe
public class ROS2BehaviorTreeSubscription<T extends BehaviorTreeNodeLayer<T, ?, ?, ?>>
{
   private final ROS2Topic<BehaviorTreeStateMessage> topic;
   private final ArrayList<Runnable> messageRecievedCallbacks = new ArrayList<>();
   private final BehaviorTreeState behaviorTreeState;
   private final Consumer<T> rootNodeSetter;
   private long numberOfMessagesReceived = 0;
   private long previousSequenceID = -1;
   private long messageDropCount = 0;
   private long outOfOrderCount = 0;
   private int numberOfOnRobotNodes = 0;
   private final SwapReference<BehaviorTreeStateMessage> behaviorTreeStateMessageSwapReference;
   private final Notification recievedMessageNotification = new Notification();
   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   public ROS2BehaviorTreeSubscription(BehaviorTreeState behaviorTreeState,
                                       Consumer<T> rootNodeSetter,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.rootNodeSetter = rootNodeSetter;

      topic = AutonomyAPI.BEAVIOR_TREE.getTopic(behaviorTreeState.getCRDTInfo().getActorDesignation().getIncomingQualifier());
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

         if (Math.abs(difference) < ROS2BehaviorTreeState.SYNC_FREQUENCY) // Account for restarts
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
            boolean subscriptionRootIsNull = behaviorTreeStateMessage.getBehaviorTreeTypes().isEmpty();
            if (!subscriptionRootIsNull)
               buildSubscriptionTree(behaviorTreeStateMessage, subscriptionRootNode);

            behaviorTreeState.fromMessage(behaviorTreeStateMessage);

            if (behaviorTreeState.isModificationIncoming())
            {
               // Clear tree to rebuild with new state
               behaviorTreeState.modifyTreeTopology(topologyOperationQueue ->
                                      topologyOperationQueue.queueOperation(behaviorTreeState.getTreeRebuilder().getClearSubtreeOperation()));
            }

            behaviorTreeState.modifyTreeTopology(topologyOperationQueue ->
            {
               T rootNode = (T) behaviorTreeState.getRootNode();

               boolean allowReplicatingRoot = behaviorTreeState.isModificationIncoming();
               if (allowReplicatingRoot)
                  rootNode = subscriptionRootIsNull ? null : retrieveOrReplicateLocalNode(subscriptionRootNode, allowReplicatingRoot);

               topologyOperationQueue.queueSetRootNode(rootNode, rootNodeSetter);

               if (rootNode != null && !subscriptionRootIsNull)
                  retrieveOrReplicateSubreeFromSubscription(subscriptionRootNode, rootNode, topologyOperationQueue);

               topologyOperationQueue.queueOperation(behaviorTreeState.getTreeRebuilder().getDestroyLeftoversOperation());
            });
         }
      }
   }

   private void retrieveOrReplicateSubreeFromSubscription(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                          T localNode,
                                                          BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      // Update the node first, to detect incoming modifications
      ROS2BehaviorTreeMessageTools.fromMessage(subscriptionNode, localNode.getState());

      for (int i = 0; i < subscriptionNode.getChildren().size(); i++)
      {
         boolean allowReplication = localNode.getDefinition().isModificationIncoming();
         T localChildNode = (T) retrieveOrReplicateLocalNode(subscriptionNode.getChildren().get(i), allowReplication);

         if (localChildNode != null)
         {
            topologyOperationQueue.queueAddNode(localChildNode, localNode);
            retrieveOrReplicateSubreeFromSubscription(subscriptionNode.getChildren().get(i), localChildNode, topologyOperationQueue);
         }
      }
   }

   private T retrieveOrReplicateLocalNode(ROS2BehaviorTreeSubscriptionNode subscriptionNode, boolean allowReplication)
   {
      long nodeID = subscriptionNode.getBehaviorTreeNodeStateMessage().getId();
      T localNode = (T) behaviorTreeState.getTreeRebuilder().getReplacementNode(nodeID);
      if (localNode == null && allowReplication) // New node that wasn't in the local tree; duplicate of one on the other side
      {
         LogTools.info("Replicating node: %s:%d (%s) Actor: %s".formatted(
               subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getName(),
               nodeID,
               subscriptionNode.getType().getSimpleName(),
               behaviorTreeState.getCRDTInfo().getActorDesignation().name()));
         localNode = (T) behaviorTreeState.getNodeStateBuilder()
                                          .createNode(subscriptionNode.getType(),
                                                      nodeID,
                                                      behaviorTreeState.getCRDTInfo(),
                                                      behaviorTreeState.getSaveFileDirectory());
      }

      return localNode;
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
