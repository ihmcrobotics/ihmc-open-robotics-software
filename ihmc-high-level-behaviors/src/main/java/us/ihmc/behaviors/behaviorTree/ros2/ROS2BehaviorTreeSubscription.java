package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;

import java.util.function.BiFunction;

public class ROS2BehaviorTreeSubscription
{
   public ROS2BehaviorTreeSubscription(ROS2BehaviorTree ros2BehaviorTree,
                                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                       ROS2IOTopicQualifier incomingQualifier,
                                       BiFunction<SceneGraph, ROS2BehaviorTreeSubscriptionNode, SceneNode> newNodeSupplier)
   {

   }

   public void update()
   {


   }

   public void destroy()
   {


   }
}
