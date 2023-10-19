package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;

   public ROS2BehaviorTreeExecutor(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                   ROS2ActorDesignation ros2ActorDesignation)
   {
      super(new ROS2BehaviorTreeState(ros2PublishSubscribeAPI, ros2ActorDesignation));

      ros2BehaviorTreeState = (ROS2BehaviorTreeState) getBehaviorTreeState();
   }

   public void update()
   {
      ros2BehaviorTreeState.updateSubscription();

      super.update();

      ros2BehaviorTreeState.updatePublication();
   }
}
