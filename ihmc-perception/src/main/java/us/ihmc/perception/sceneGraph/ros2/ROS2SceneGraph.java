package us.ihmc.perception.sceneGraph.ros2;

import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.tools.thread.Throttler;

import java.util.function.BiFunction;

/**
 * Implements the scene graph CRDT with ROS 2 messages.
 */
public class ROS2SceneGraph extends SceneGraph
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ROS2IOTopicQualifier subscriptionQualifier;
   private final ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   /**
    * Constructor for on-robot.
    */
   public ROS2SceneGraph(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME),
           (sceneGraph, subscriptionNode) -> ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph),
           ros2PublishSubscribeAPI,
           ROS2IOTopicQualifier.COMMAND);
   }

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2SceneGraph(SceneNode rootNode,
                         BiFunction<ROS2SceneGraph, ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier,
                         ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                         ROS2IOTopicQualifier subscriptionQualifier)
   {
      super(rootNode);
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.subscriptionQualifier = subscriptionQualifier;

      sceneGraphSubscription = new ROS2SceneGraphSubscription(this, ros2PublishSubscribeAPI, subscriptionQualifier, newNodeSupplier);
   }

   public void updateSubscription()
   {
      sceneGraphSubscription.update();
   }

   public void updatePublication()
   {
      if (publishThrottler.run())
         sceneGraphPublisher.publish(this, ros2PublishSubscribeAPI, subscriptionQualifier.getOpposite());
   }

   public void destroy()
   {
      sceneGraphSubscription.destroy();
   }

   public ROS2SceneGraphSubscription getSceneGraphSubscription()
   {
      return sceneGraphSubscription;
   }
}
