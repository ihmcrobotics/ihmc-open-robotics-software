package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.tools.thread.Throttler;

import java.util.function.Consumer;

/**
 * This class is concerned with syncing behavior tree state only
 * over ROS 2 nodes as a CRDT.
 */
public class ROS2BehaviorTreeState
{
   private final BehaviorTreeState behaviorTreeState;
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ROS2ActorDesignation ros2ActorDesignation;
   private final ROS2BehaviorTreeSubscription behaviorTreeSubscription;
   private final ROS2BehaviorTreePublisher behaviorTreePublisher = new ROS2BehaviorTreePublisher();

   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2BehaviorTreeState(BehaviorTreeState behaviorTreeState,
                                Consumer<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSetter,
                                ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                ROS2ActorDesignation ros2ActorDesignation)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.ros2ActorDesignation = ros2ActorDesignation;

      behaviorTreeSubscription = new ROS2BehaviorTreeSubscription(behaviorTreeState,
                                                                  rootNodeSetter,
                                                                  ros2PublishSubscribeAPI,
                                                                  ros2ActorDesignation.getIncomingQualifier());
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
      if (publishThrottler.run())
         behaviorTreePublisher.publish(behaviorTreeState, ros2PublishSubscribeAPI, ros2ActorDesignation.getOutgoingQualifier());
   }

   public void destroy()
   {
      behaviorTreeSubscription.destroy();
   }

   public BehaviorTreeState getBehaviorTreeState()
   {
      return behaviorTreeState;
   }

   public ROS2BehaviorTreeSubscription getBehaviorTreeSubscription()
   {
      return behaviorTreeSubscription;
   }
}
