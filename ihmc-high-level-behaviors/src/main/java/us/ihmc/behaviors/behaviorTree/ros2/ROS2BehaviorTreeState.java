package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.function.Consumer;

/**
 * This class is concerned with syncing behavior tree state only
 * over ROS 2 nodes as a CRDT.
 *
 * @param <L> Root node layer type.
 * @param <S> Root node state type.
 * @param <D> Root node definition type.
 */
public class ROS2BehaviorTreeState<L extends BehaviorTreeNodeLayer<?, S, S, D>,
                                   S extends BehaviorTreeNodeState<D>,
                                   D extends BehaviorTreeNodeDefinition>
{
   /**
    * The SYNC_FREQUENCY should be a multiple of the scene graph's update frequency.
    */
   public static final double SYNC_FREQUENCY = SceneGraph.UPDATE_FREQUENCY / 2.0;

   private final BehaviorTreeState<L, S, D> behaviorTreeState;
   private final ROS2BehaviorTreePublisher behaviorTreePublisher;
   private final ROS2BehaviorTreeSubscription<L, S, D> behaviorTreeSubscription;

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2BehaviorTreeState(BehaviorTreeState<L, S, D> behaviorTreeState,
                                Consumer<L> rootNodeSetter,
                                ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTreeState = behaviorTreeState;

      behaviorTreePublisher = new ROS2BehaviorTreePublisher(behaviorTreeState, ros2PublishSubscribeAPI);
      behaviorTreeSubscription = new ROS2BehaviorTreeSubscription<>(behaviorTreeState, rootNodeSetter, ros2PublishSubscribeAPI);
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
      behaviorTreeState.getCRDTInfo().startNextUpdate();
   }

   public void destroy()
   {
      behaviorTreeSubscription.destroy();
   }

   public BehaviorTreeState<L, S, D> getBehaviorTreeState()
   {
      return behaviorTreeState;
   }

   public ROS2BehaviorTreeSubscription getBehaviorTreeSubscription()
   {
      return behaviorTreeSubscription;
   }
}
