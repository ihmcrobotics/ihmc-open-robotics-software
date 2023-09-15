package us.ihmc.perception.sceneGraph.ros2;

import org.apache.commons.lang3.mutable.MutableInt;
import perception_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

import java.util.function.Function;

/**
 * Subscribes to, synchronizing, the robot's perception scene graph.
 */
public class ROS2SceneGraphSubscription
{
   private final IHMCROS2Input<SceneGraphMessage> sceneGraphSubscription;
   private final SceneGraph sceneGraph;
   private final Function<ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier;
   private final ROS2IOTopicQualifier ioQualifier;
   private final FramePose3D nodePose = new FramePose3D();
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private long numberOfMessagesReceived = 0;
   private boolean localTreeFrozen = false;
   private SceneGraphMessage latestSceneGraphMessage;

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    */
   public ROS2SceneGraphSubscription(SceneGraph sceneGraph,
                                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2IOTopicQualifier ioQualifier)
   {
      this(sceneGraph,
           ros2PublishSubscribeAPI,
           ioQualifier,
           ros2SceneGraphSubscriptionNode -> ROS2SceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode, sceneGraph));
   }

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    * @param newNodeSupplier So that new nodes can be externally extended, like for UI representations.
    *                        Use {@link ROS2SceneGraphTools#createNodeFromMessage} as a base.
    */
   public ROS2SceneGraphSubscription(SceneGraph sceneGraph,
                                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2IOTopicQualifier ioQualifier,
                                     Function<ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier)
   {
      this.sceneGraph = sceneGraph;
      this.newNodeSupplier = newNodeSupplier;
      this.ioQualifier = ioQualifier;

      sceneGraphSubscription = ros2PublishSubscribeAPI.subscribe(PerceptionAPI.SCENE_GRAPH.getTopic(ioQualifier));
   }

   /**
    * Check for a new ROS 2 message and update the scene nodes with it.
    * This method runs on the robot and on every connected UI.
    * @return if a new message was used to update the scene nodes on this call
    */
   public boolean update()
   {
      boolean newMessageAvailable = sceneGraphSubscription.getMessageNotification().poll();
      if (newMessageAvailable)
      {
         ++numberOfMessagesReceived;
         latestSceneGraphMessage = sceneGraphSubscription.getMessageNotification().read();

         ROS2SceneGraphSubscriptionNode subscriptionRootNode = new ROS2SceneGraphSubscriptionNode();
         buildSubscriptionTree(new MutableInt(), latestSceneGraphMessage, subscriptionRootNode);

         // If the tree was recently modified by the operator, we do not accept
         // updates the structure of the tree.
         localTreeFrozen = false;
         checkTreeModified(sceneGraph.getRootNode());

         if (!localTreeFrozen)
            clearChildren(sceneGraph.getRootNode());
         updateLocalTreeFromSubscription(subscriptionRootNode, sceneGraph.getRootNode(), ReferenceFrame.getWorldFrame());

         sceneGraph.updateCaches();
      }
      return newMessageAvailable;
   }

   private void updateLocalTreeFromSubscription(ROS2SceneGraphSubscriptionNode subscriptionNode, SceneNode localNode, ReferenceFrame parentFrame)
   {
      // If tree is frozen and the ID isn't in the local tree, we don't have anything to update
      // This'll happen if a node is deleted locally. We want that change to propagate and not add
      // it right back.
      if (localNode instanceof DetectableSceneNode detectableSceneNode)
      {
         detectableSceneNode.setCurrentlyDetected(subscriptionNode.getDetectableSceneNodeMessage().getCurrentlyDetected());
      }
      if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.setCurrentDistance(subscriptionNode.getStaticRelativeSceneNodeMessage().getCurrentDistanceToRobot());
      }

      // If the node was recently modified by the operator, the node does not accept
      // updates of these values. This is to allow the operator's changes to propagate
      // and so it doesn't get overriden immediately by an out of date message coming from the robot.
      // On the robot side, this will always get updated because there is no operator.
      if (!localTreeFrozen)
      {
         if (localNode instanceof ArUcoMarkerNode arUcoMarkerNode)
         {
            arUcoMarkerNode.setMarkerID(subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId());
            arUcoMarkerNode.setMarkerSize(subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize());
            arUcoMarkerNode.setBreakFrequency(subscriptionNode.getArUcoMarkerNodeMessage().getBreakFrequency());
         }
         if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
         {
            staticRelativeSceneNode.setDistanceToDisableTracking(subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking());
         }

         localNode.ensureParentFrameEquals(parentFrame);

         MessageTools.toEuclid(subscriptionNode.getSceneNodeMessage().getTransformToWorld(), nodeToWorldTransform);
         nodePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), nodeToWorldTransform);
         nodePose.changeFrame(localNode.getNodeFrame().getParent());
         nodePose.get(localNode.getNodeToParentFrameTransform());
         localNode.getNodeFrame().update();
      }

      for (ROS2SceneGraphSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         SceneNode localChildNode = sceneGraph.getIDToNodeMap().get(subscriptionChildNode.getSceneNodeMessage().getId());
         if (localChildNode == null && !localTreeFrozen) // New node that wasn't in the local tree
         {
            localChildNode = newNodeSupplier.apply(subscriptionChildNode);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode.getNodeFrame());

            if (!localTreeFrozen)
               localNode.getChildren().add(localChildNode);
         }
      }
   }

   private void checkTreeModified(SceneNode localNode)
   {
      localTreeFrozen |= localNode.operatorModifiedThisRecently();

      for (SceneNode child : localNode.getChildren())
      {
         checkTreeModified(child);
      }
   }

   private void clearChildren(SceneNode localNode)
   {
      for (SceneNode child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();
   }

   private void buildSubscriptionTree(MutableInt index, SceneGraphMessage sceneGraphMessage, ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      byte sceneNodeType = sceneGraphMessage.getSceneTreeTypes().get(index.intValue());
      int indexInTypesList = (int) sceneGraphMessage.getSceneTreeIndices().get(index.intValue());
      subscriptionNode.setType(sceneNodeType);

      switch (sceneNodeType)
      {
         case SceneGraphMessage.SCENE_NODE_TYPE ->
         {
            subscriptionNode.setSceneNodeMessage(sceneGraphMessage.getSceneNodes().get(indexInTypesList));
         }
         case SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE ->
         {
            DetectableSceneNodeMessage detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().get(indexInTypesList);
            subscriptionNode.setDetectableSceneNodeMessage(detectableSceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(detectableSceneNodeMessage.getSceneNode());
         }
         case SceneGraphMessage.PREDEFINED_RIGID_BODY_NODE_TYPE ->
         {
            PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage
                  = sceneGraphMessage.getPredefinedRigidBodySceneNodes().get(indexInTypesList);
            subscriptionNode.setPredefinedRigidBodySceneNodeMessage(predefinedRigidBodySceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(predefinedRigidBodySceneNodeMessage.getSceneNode());
         }
         case SceneGraphMessage.ARUCO_MARKER_NODE_TYPE ->
         {
            ArUcoMarkerNodeMessage arUcoMarkerNodeMessage = sceneGraphMessage.getArucoMarkerSceneNodes().get(indexInTypesList);
            subscriptionNode.setArUcoMarkerNodeMessage(arUcoMarkerNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(arUcoMarkerNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(arUcoMarkerNodeMessage.getDetectableSceneNode().getSceneNode());
         }
         case SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE ->
         {
            StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage = sceneGraphMessage.getStaticRelativeSceneNodes().get(indexInTypesList);
            subscriptionNode.setStaticRelativeSceneNodeMessage(staticRelativeSceneNodeMessage);
            subscriptionNode.setPredefinedRigidBodySceneNodeMessage(staticRelativeSceneNodeMessage.getPredefinedRigidBodySceneNode());
            subscriptionNode.setSceneNodeMessage(staticRelativeSceneNodeMessage.getPredefinedRigidBodySceneNode().getSceneNode());
         }
      }

      for (int i = 0; i < subscriptionNode.getSceneNodeMessage().getNumberOfChildren(); i++)
      {
         ROS2SceneGraphSubscriptionNode subscriptionTreeChildNode = new ROS2SceneGraphSubscriptionNode();
         index.increment();
         buildSubscriptionTree(index, sceneGraphMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
   }

   public void destroy()
   {
      sceneGraphSubscription.destroy();
   }

   public IHMCROS2Input<SceneGraphMessage> getSceneGraphSubscription()
   {
      return sceneGraphSubscription;
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }

   public SceneGraphMessage getLatestSceneGraphMessage()
   {
      return latestSceneGraphMessage;
   }

   public boolean getLocalTreeFrozen()
   {
      return localTreeFrozen;
   }
}
