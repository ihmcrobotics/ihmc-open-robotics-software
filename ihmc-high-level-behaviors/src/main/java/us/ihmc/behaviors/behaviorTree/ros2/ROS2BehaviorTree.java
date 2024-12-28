package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.function.Consumer;

/**
 * Manages syncing a behavior tree over ROS 2 as a CRDT.
 *
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 */
public class ROS2BehaviorTree<HLT extends BehaviorTreeNodeHighLayer<HLT, ? ,?>>
{
   /**
    * The SYNC_FREQUENCY should be a multiple of the scene graph's update frequency.
    */
   public static final double SYNC_FREQUENCY = SceneGraph.UPDATE_FREQUENCY / 2.0;

   private final BehaviorTree<HLT> behaviorTree;
   private final ROS2BehaviorTreePublisher behaviorTreePublisher;
   private final ROS2BehaviorTreeSubscription<HLT> behaviorTreeSubscription;

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2BehaviorTree(BehaviorTree<HLT> behaviorTree,
                           Consumer<HLT> rootNodeSetter,
                           ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTree = behaviorTree;

      behaviorTreePublisher = new ROS2BehaviorTreePublisher(behaviorTree, ros2PublishSubscribeAPI);
      behaviorTreeSubscription = new ROS2BehaviorTreeSubscription<>(behaviorTree, rootNodeSetter, ros2PublishSubscribeAPI);
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
      behaviorTreePublisher.publish();

      // We increment the CRDT update number once per publication,
      // which done at the SYNC_FREQUENCY.
      behaviorTree.getCRDTInfo().startNextUpdate();
   }

   public void destroy()
   {
      behaviorTreeSubscription.destroy();
   }

   public BehaviorTree<HLT> getBehaviorTree()
   {
      return behaviorTree;
   }

   public ROS2BehaviorTreeSubscription<HLT> getBehaviorTreeSubscription()
   {
      return behaviorTreeSubscription;
   }
}
