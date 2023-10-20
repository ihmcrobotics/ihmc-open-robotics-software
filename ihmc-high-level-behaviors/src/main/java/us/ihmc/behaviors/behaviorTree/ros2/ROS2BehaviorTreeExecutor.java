package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{

   private final ROS2BehaviorTreeState ros2BehaviorTreeState;

   private final BehaviorTreeNodeExecutor rootNode;

   public BehaviorTreeExecutor(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                               ROS2ActorDesignation ros2ActorDesignation)
   {
      ros2BehaviorTreeState = new ROS2BehaviorTreeState(ros2PublishSubscribeAPI,
                                                        ros2ActorDesignation);

   }

   public void update()
   {
      ros2BehaviorTreeState.updateSubscription();

      rootNode.clock();

      rootNode.tick();


      ros2BehaviorTreeState.updatePublication();

   }
}
