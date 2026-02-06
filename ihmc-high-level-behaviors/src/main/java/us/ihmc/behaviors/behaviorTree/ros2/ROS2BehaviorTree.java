package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

/**
 * Manages syncing a behavior tree over ROS 2 as a CRDT.
 *
 * @param <T> The generic type of this node: RDX or Executor
 */
public class ROS2BehaviorTree<T extends BehaviorTreeNode<T, ? ,?>>
{
   public static final double SYNC_FREQUENCY = 30.0;

   private final BehaviorTree<?, ?> behaviorTree;
   private final ROS2PublishSubscribeAPI ros2;

   private final ROS2Topic<BehaviorTreeStateMessage> publishTopic;
   private final BehaviorTreeStateMessage publishMessage = new BehaviorTreeStateMessage();

   private final ROS2BehaviorTreeSubscription<T> behaviorTreeSubscription;

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2BehaviorTree(BehaviorTree<BehaviorTreeRootNode<T>, T> behaviorTree, ROS2PublishSubscribeAPI ros2)
   {
      this.behaviorTree = behaviorTree;
      this.ros2 = ros2;

      publishTopic = AutonomyAPI.BEHAVIOR_TREE.getTopic(behaviorTree.getCRDTInfo().getActorDesignation().getOutgoingQualifier());
      behaviorTreeSubscription = new ROS2BehaviorTreeSubscription<>(behaviorTree, ros2);
   }

   /**
    * Call before performing operations on the behavior tree once per tick of your thread.
    * This gets the behavior tree up-to-date with the latest information.
    *
    * This method is separate from the updatePublication because you want to publish after doing
    * a local possible modification of the behavior tree first. Additionally, some processes
    * just need to have a read-only copy of the behavior tree, such as autonomy processes that
    * merely act in the environment.
    */
   public void updateSubscription()
   {
      behaviorTreeSubscription.update();
   }

   /**
    * Publishes the behavior tree to the other side, whether that be the UI or the robot's
    * behavior tree instance. Call this closer to the end of your thread's tick, after
    * performing possible local modifications.
    */
   public void updatePublication()
   {
      behaviorTree.toMessage(publishMessage);
      ros2.publish(publishTopic, publishMessage);

      // We increment the CRDT update number once per publication,
      // which done at the SYNC_FREQUENCY.
      behaviorTree.getCRDTInfo().startNextUpdate();
   }

   public void destroy()
   {
      behaviorTreeSubscription.destroy();
   }

   public ROS2BehaviorTreeSubscription<T> getBehaviorTreeSubscription()
   {
      return behaviorTreeSubscription;
   }
}
