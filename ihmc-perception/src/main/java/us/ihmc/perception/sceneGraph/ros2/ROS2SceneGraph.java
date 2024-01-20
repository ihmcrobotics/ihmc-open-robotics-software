package us.ihmc.perception.sceneGraph.ros2;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.thread.Throttler;

import java.util.function.BiFunction;

/**
 * Implements the scene graph CRDT with ROS 2 messages.
 */
public class ROS2SceneGraph extends SceneGraph
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ROS2ActorDesignation ros2ActorDesignation;
   private final ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   /**
    * Constructor for on-robot.
    */
   public ROS2SceneGraph(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME), null, ros2PublishSubscribeAPI, ROS2ActorDesignation.ROBOT, referenceFrameLibrary);
   }

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public ROS2SceneGraph(SceneNode rootNode,
                         BiFunction<SceneGraph, ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier,
                         ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                         ROS2ActorDesignation ros2ActorDesignation,
                         ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(rootNode, referenceFrameLibrary);
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.ros2ActorDesignation = ros2ActorDesignation;

      sceneGraphSubscription = new ROS2SceneGraphSubscription(this, ros2PublishSubscribeAPI, ros2ActorDesignation.getIncomingQualifier(), newNodeSupplier);
   }

   /**
    * Call before performing operations on the scene graph once per tick of your thread.
    * This gets the scene graph up-to-date with the latest information.
    *
    * This method is separate from the updatePublication because you want to publish after doing
    * a local possible modification of the scene graph first. Additionally, some processes
    * just need to have a read-only copy of the scene graph, such as autonomy processes that
    * merely act in the environment.
    */
   public void updateSubscription()
   {
      sceneGraphSubscription.update();
   }

   /**
    * Publishes the scene graph to the other side, whether that be the UI or the robot's
    * scene graph instance. Call this closer to the end of your thread's tick, after
    * performing possible local modifications.
    */
   public void updatePublication()
   {
      if (publishThrottler.run())
         sceneGraphPublisher.publish(this, ros2PublishSubscribeAPI, ros2ActorDesignation.getOutgoingQualifier());
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
