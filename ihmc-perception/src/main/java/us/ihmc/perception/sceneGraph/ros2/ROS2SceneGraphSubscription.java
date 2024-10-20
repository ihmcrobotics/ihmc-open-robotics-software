package us.ihmc.perception.sceneGraph.ros2;

import org.apache.commons.lang3.mutable.MutableInt;
import perception_msgs.msg.dds.ArUcoMarkerNodeMessage;
import perception_msgs.msg.dds.CenterposeNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.DoorNodeMessage;
import perception_msgs.msg.dds.DoorOpeningMechanismMessage;
import perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import perception_msgs.msg.dds.StaticRelativeSceneNodeMessage;
import perception_msgs.msg.dds.TrashCanNodeMessage;
import perception_msgs.msg.dds.YOLOv8NodeMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeReplacement;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism.DoorOpeningMechanismType;
import us.ihmc.perception.sceneGraph.rigidBody.trashcan.TrashCanNode;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.UUID;
import java.util.function.BiFunction;

/**
 * Subscribes to, synchronizing, the robot's perception scene graph.
 */
public class ROS2SceneGraphSubscription
{
   private final SwapReference<SceneGraphMessage> sceneGraphMessageSwapReference;
   private final Notification recievedMessageNotification = new Notification();
   private final ArrayList<Runnable> messageRecievedCallbacks = new ArrayList<>();
   private final SceneGraph sceneGraph;
   private final BiFunction<SceneGraph, ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier;
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();
   private long numberOfMessagesReceived = 0;
   private long previousUpdateNumber = -1;
   private long messageDropCount = 0;
   private int numberOfOnRobotNodes = 0;
   private boolean localTreeFrozen = false;
   private final ROS2SceneGraphSubscriptionNode subscriptionRootNode = new ROS2SceneGraphSubscriptionNode();
   private final MutableInt subscriptionNodeDepthFirstIndex = new MutableInt();

   public ROS2SceneGraphSubscription(SceneGraph sceneGraph, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ROS2IOTopicQualifier ioQualifier)
   {
      this(sceneGraph, ros2PublishSubscribeAPI, ioQualifier, null);
   }

   /**
    * @param ioQualifier If in the on-robot perception process, COMMAND, else STATUS
    * @param newNodeSupplier So that new nodes can be externally extended, like for UI representations.
    *                        Use {@link ROS2SceneGraphTools#createNodeFromMessage} as a base.
    */
   public ROS2SceneGraphSubscription(SceneGraph sceneGraph,
                                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2IOTopicQualifier ioQualifier,
                                     BiFunction<SceneGraph, ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier)
   {
      this.sceneGraph = sceneGraph;
      if (newNodeSupplier != null)
         this.newNodeSupplier = newNodeSupplier;
      else
         this.newNodeSupplier = (uneeded, subscriptionNode) -> ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph);

      sceneGraphMessageSwapReference = ros2PublishSubscribeAPI.subscribeViaSwapReference(PerceptionAPI.SCENE_GRAPH.getTopic(ioQualifier),
                                                                                         recievedMessageNotification);
   }

   /**
    * Check for a new ROS 2 message and update the scene nodes with it.
    * This method runs on the robot and on every connected UI.
    * @return if a new message was used to update the scene nodes on this call
    */
   public void update()
   {
      if (recievedMessageNotification.poll())
      {
         synchronized (sceneGraphMessageSwapReference)
         {
            SceneGraphMessage sceneGraphMessage = sceneGraphMessageSwapReference.getForThreadTwo();

            numberOfOnRobotNodes = sceneGraphMessage.getSceneTreeIndices().size();

            ++numberOfMessagesReceived;
            for (Runnable messageRecievedCallback : messageRecievedCallbacks)
            {
               messageRecievedCallback.run();
            }

            long nextUpdateNumber = sceneGraphMessage.getSequenceId();
            if (previousUpdateNumber > -1)
            {
               long expectedUpdateNumber = previousUpdateNumber + 1;
               messageDropCount += nextUpdateNumber - expectedUpdateNumber;
            }
            previousUpdateNumber = nextUpdateNumber;

            subscriptionRootNode.clear();
            subscriptionNodeDepthFirstIndex.setValue(0);
            buildSubscriptionTree(sceneGraphMessage, subscriptionRootNode);

            // If the tree was recently modified by the operator, we do not accept
            // updates the structure of the tree.
            localTreeFrozen = false;
            checkTreeModified(sceneGraph.getRootNode());

            if (!localTreeFrozen)
               sceneGraph.getNextID().set(sceneGraphMessage.getNextId());

            sceneGraph.modifyTree(modificationQueue ->
            {
               if (!localTreeFrozen)
                  modificationQueue.accept(new SceneGraphClearSubtree(sceneGraph.getRootNode()));

               updateLocalTreeFromSubscription(subscriptionRootNode, sceneGraph.getRootNode(), null, modificationQueue);
            });
         }
      }
   }

   private void updateLocalTreeFromSubscription(ROS2SceneGraphSubscriptionNode subscriptionNode,
                                                SceneNode localNode,
                                                SceneNode localParentNode,
                                                SceneGraphModificationQueue modificationQueue)
   {
      // Set fields only modifiable by the robot
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
            // TODO: FIXME TOMASZ PLEASE ADD INSTANT DETECTION UPDATE
         }
         if (localNode instanceof CenterposeNode centerposeNode)
         {
            centerposeNode.setConfidence(subscriptionNode.getCenterposeNodeMessage().getConfidence());
            centerposeNode.setBoundingBoxVertices(subscriptionNode.getCenterposeNodeMessage().getBoundingBoxVertices());
            centerposeNode.setBoundingBoxVertices2D(Arrays.stream(subscriptionNode.getCenterposeNodeMessage().getBoundingBoxVertices2d())
                                                          .map(Point2D::new)
                                                          .toArray(Point2D[]::new));
            centerposeNode.setEnableTracking(subscriptionNode.getCenterposeNodeMessage().getEnableTracking());
         }
         if (localNode instanceof YOLOv8Node yoloNode)
         {
            yoloNode.fromMessage(subscriptionNode.getYOLONodeMessage());
         }
         if (localNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
         {
            staticRelativeSceneNode.setDistanceToDisableTracking(subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking());
         }
         if (localNode instanceof DoorNode doorNode)
         {
            doorNode.updateDoorCornerFrame(subscriptionNode.getDoorNodeMessage().getDoorCornerTransformToWorld());
            doorNode.setDoorFramePoseLock(subscriptionNode.getDoorNodeMessage().getPoseLocked());
            doorNode.getDoorPanel().fromMessage(subscriptionNode.getDoorNodeMessage().getDoorPanel());
            for (DoorOpeningMechanismMessage doorOpeningMechanismMessage : subscriptionNode.getDoorNodeMessage().getOpeningMechanisms())
            {
               UUID messageDetectionID = MessageTools.toUUID(doorOpeningMechanismMessage.getPersistentDetectionId());
               if (!messageDetectionID.equals(PersistentDetection.NULL_DETECTION_ID))
               {
                  if (!doorNode.getOpeningMechanisms().containsKey(messageDetectionID))
                  {
                     DoorSide doorSide = DoorSide.fromBoolean(doorOpeningMechanismMessage.getDoorSide());
                     DoorOpeningMechanismType openingMechanismType = DoorOpeningMechanismType.fromByte(doorOpeningMechanismMessage.getType());
                     DoorOpeningMechanism doorOpeningMechanism = new DoorOpeningMechanism(doorSide, openingMechanismType, messageDetectionID);
                     doorOpeningMechanism.updateMechanismFrame(doorOpeningMechanismMessage.getMechanismTransformToWorld());
                     doorNode.getOpeningMechanisms().put(messageDetectionID, doorOpeningMechanism);
                  }
                  else
                  {
                     doorNode.getOpeningMechanisms().get(messageDetectionID).updateMechanismFrame(doorOpeningMechanismMessage.getMechanismTransformToWorld());
                  }
               }
            }
         }
         if (localNode instanceof TrashCanNode trashCanNode)
         {
            trashCanNode.updateFromMessage(subscriptionNode.getTrashCanNodeMessage());
         }

         if (localParentNode != null) // Parent of root node is null
         {
            MessageTools.toEuclid(subscriptionNode.getSceneNodeMessage().getTransformToWorld(), nodeToWorldTransform);
            modificationQueue.accept(new SceneGraphNodeReplacement(localNode, localParentNode, nodeToWorldTransform));
         }
      }

      for (ROS2SceneGraphSubscriptionNode subscriptionChildNode : subscriptionNode.getChildren())
      {
         SceneNode localChildNode = sceneGraph.getIDToNodeMap().get(subscriptionChildNode.getSceneNodeMessage().getId());
         if (localChildNode == null && !localTreeFrozen) // New node that wasn't in the local tree
         {
            localChildNode = newNodeSupplier.apply(sceneGraph, subscriptionChildNode);
         }

         if (localChildNode != null)
         {
            updateLocalTreeFromSubscription(subscriptionChildNode, localChildNode, localNode, modificationQueue);
         }
      }

      // Update the state after iterating over children, because node can unfreeze at this time
      localNode.fromMessage(subscriptionNode.getSceneNodeMessage().getConfirmableRequest());
   }

   private void checkTreeModified(SceneNode localNode)
   {
      localTreeFrozen |= localNode.isFrozen();

      for (SceneNode child : localNode.getChildren())
      {
         checkTreeModified(child);
      }
   }

   /** Build an intermediate tree representation of the message, which helps to sync with the actual tree. */
   private void buildSubscriptionTree(SceneGraphMessage sceneGraphMessage, ROS2SceneGraphSubscriptionNode subscriptionNode)
   {
      byte sceneNodeType = sceneGraphMessage.getSceneTreeTypes().get(subscriptionNodeDepthFirstIndex.intValue());
      int indexInTypesList = (int) sceneGraphMessage.getSceneTreeIndices().get(subscriptionNodeDepthFirstIndex.intValue());
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
         case SceneGraphMessage.CENTERPOSE_NODE_TYPE ->
         {
            CenterposeNodeMessage centerposeNodeMessage = sceneGraphMessage.getCenterposeSceneNodes().get(indexInTypesList);
            subscriptionNode.setCenterposeNodeMessage(centerposeNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(centerposeNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(centerposeNodeMessage.getDetectableSceneNode().getSceneNode());
         }
         case SceneGraphMessage.YOLO_NODE_TYPE ->
         {
            YOLOv8NodeMessage yoloNodeMessage = sceneGraphMessage.getYoloSceneNodes().get(indexInTypesList);
            subscriptionNode.setYOLONodeMessage(yoloNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(yoloNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(yoloNodeMessage.getDetectableSceneNode().getSceneNode());
         }
         case SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE ->
         {
            StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage = sceneGraphMessage.getStaticRelativeSceneNodes().get(indexInTypesList);
            subscriptionNode.setStaticRelativeSceneNodeMessage(staticRelativeSceneNodeMessage);
            subscriptionNode.setPredefinedRigidBodySceneNodeMessage(staticRelativeSceneNodeMessage.getPredefinedRigidBodySceneNode());
            subscriptionNode.setSceneNodeMessage(staticRelativeSceneNodeMessage.getPredefinedRigidBodySceneNode().getSceneNode());
         }
         case SceneGraphMessage.PRIMITIVE_RIGID_BODY_NODE_TYPE ->
         {
            PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage = sceneGraphMessage.getPrimitiveRigidBodySceneNodes().get(indexInTypesList);
            subscriptionNode.setPrimitiveRigidBodySceneNodeMessage(primitiveRigidBodySceneNodeMessage);
            subscriptionNode.setSceneNodeMessage(primitiveRigidBodySceneNodeMessage.getSceneNode());
         }
         case SceneGraphMessage.DOOR_NODE_TYPE ->
         {
            DoorNodeMessage doorNodeMessage = sceneGraphMessage.getDoorSceneNodes().get(indexInTypesList);
            subscriptionNode.setDoorNodeMessage(doorNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(doorNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(doorNodeMessage.getDetectableSceneNode().getSceneNode());
         }
         case SceneGraphMessage.TRASH_CAN_NODE_TYPE ->
         {
            TrashCanNodeMessage trashCanNodeMessage = sceneGraphMessage.getTrashCanNodes().get(indexInTypesList);
            subscriptionNode.setTrashCanNodeMessage(trashCanNodeMessage);
            subscriptionNode.setDetectableSceneNodeMessage(trashCanNodeMessage.getDetectableSceneNode());
            subscriptionNode.setSceneNodeMessage(trashCanNodeMessage.getDetectableSceneNode().getSceneNode());
         }
      }

      for (int i = 0; i < subscriptionNode.getSceneNodeMessage().getNumberOfChildren(); i++)
      {
         ROS2SceneGraphSubscriptionNode subscriptionTreeChildNode = new ROS2SceneGraphSubscriptionNode();
         subscriptionNodeDepthFirstIndex.increment();
         buildSubscriptionTree(sceneGraphMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
   }

   public void destroy()
   {

   }

   public void registerMessageReceivedCallback(Runnable callback)
   {
      messageRecievedCallbacks.add(callback);
   }

   public int getNumberOfOnRobotNodes()
   {
      return numberOfOnRobotNodes;
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }

   public long getMessageDropCount()
   {
      return messageDropCount;
   }
}
