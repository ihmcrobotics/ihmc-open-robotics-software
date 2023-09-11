package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.ArUcoMarkerNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import perception_msgs.msg.dds.StaticRelativeSceneNodeMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;

/**
 * Subscribes to, synchronizing, the robot's perception scene graph.
 */
public class ROS2SceneGraphSubscription
{
   private final IHMCROS2Input<SceneGraphMessage> sceneGraphSubscription;
   private final SceneGraph sceneGraph;
   private final FramePose3D nodePose = new FramePose3D();
   private final FramePose3D arUcoMarkerPose = new FramePose3D();
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform arUcoMarkerToWorldTransform = new RigidBodyTransform();
   private long numberOfMessagesReceived = 0;
   private boolean localTreeFrozen = false;

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    */
   public ROS2SceneGraphSubscription(SceneGraph sceneGraph,
                                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2IOTopicQualifier ioQualifier)
   {
      this.sceneGraph = sceneGraph;

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
         SceneGraphMessage sceneGraphMessage = sceneGraphSubscription.getMessageNotification().read();
         IDLSequence.Object<DetectableSceneNodeMessage> detectableSceneNodeMessages = sceneGraphMessage.getDetectableSceneNodes();

         ROS2SceneGraphSubscriptionNode subscriptionRootNode = new ROS2SceneGraphSubscriptionNode();
         buildSubscriptionTree(0, sceneGraphMessage, subscriptionRootNode);

         // If the tree was recently modified by the operator, we do not accept
         // updates the structure of the tree.
         localTreeFrozen = false;
         checkTreeModified(sceneGraph.getRootNode());

         sceneGraph.updateCaches();
         if (!localTreeFrozen)
            clearChildren(sceneGraph.getRootNode());
         updateLocalTreeFromSubscription(subscriptionRootNode);
      }
      return newMessageAvailable;
   }

   private SceneNode updateLocalTreeFromSubscription(ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      SceneNode localNode = sceneGraph.getIDToNodeMap().get(subscriptionNode.getSceneNodeMessage().getId());

      if (!localTreeFrozen && localNode == null) // New node that wasn't in the local tree
      {
         localNode = ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode);
      }

      if (localNode instanceof DetectableSceneNode detectableSceneNode)
      {
         detectableSceneNode.setCurrentlyDetected(subscriptionNode.getDetectableSceneNodeMessage().getCurrentlyDetected());
      }
      if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.setCurrentDistance(subscriptionNode.getStaticRelativeSceneNodeMessageeMessage().getCurrentDistanceToRobot());
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
            staticRelativeSceneNode.setDistanceToDisableTracking(subscriptionNode.getStaticRelativeSceneNodeMessageeMessage().getDistanceToDisableTracking());
         }

         MessageTools.toEuclid(subscriptionNode.getSceneNodeMessage().getTransformToWorld(), nodeToWorldTransform);
         nodePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), nodeToWorldTransform);
         nodePose.changeFrame(localNode.getNodeFrame().getParent());
         nodePose.get(localNode.getNodeToParentFrameTransform());
         localNode.getNodeFrame().update();
      }

      for (ROS2SceneGraphSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         SceneNode localChildNode = updateLocalTreeFromSubscription(subscriptionChildNode);

         if (!localTreeFrozen)
            localNode.getChildren().add(localChildNode);
      }

      return localNode;
   }

   private void checkTreeModified(SceneNode localNode)
   {
      localTreeFrozen |= localNode.operatorHasntModifiedThisRecently();

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

   private void buildSubscriptionTree(int index, SceneGraphMessage sceneGraphMessage, ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      byte sceneNodeType = sceneGraphMessage.getSceneTreeTypes().get(index);
      int indexInTypesList = (int) sceneGraphMessage.getSceneTreeIndices().get(index);
      switch (sceneNodeType)
      {
         case SceneGraphMessage.SCENE_NODE_TYPE ->
               subscriptionNode.setSceneNodeMessage(sceneGraphMessage.getSceneNodes().get(indexInTypesList));
         case SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE ->
         {
            DetectableSceneNodeMessage detectableSceneNodeMessage = sceneGraphMessage.getDetectableSceneNodes().get(indexInTypesList);
            subscriptionNode.setDetectableSceneNodeMessage(detectableSceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(detectableSceneNodeMessage.getSceneNode());
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
            subscriptionNode.setStaticRelativeSceneNodeMessageeMessage(staticRelativeSceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(staticRelativeSceneNodeMessage.getSceneNode());
         }
      }

      for (int i = 0; i < subscriptionNode.getSceneNodeMessage().getNumberOfChildren(); i++)
      {
         ROS2SceneGraphSubscriptionNode subscriptionTreeChildNode = new ROS2SceneGraphSubscriptionNode();
         buildSubscriptionTree(index++, sceneGraphMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
   }

   public void destroy()
   {
      sceneGraphSubscription.destroy();
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }
}
