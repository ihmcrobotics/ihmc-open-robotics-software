package us.ihmc.behaviors.behaviorTree.control.door;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTBidirectionalDouble lostGraspDetectionHandOpenAngle;
   private final CRDTBidirectionalDouble openedDoorHandleDistanceFromStart;

   public DoorTraversalDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      lostGraspDetectionHandOpenAngle = new CRDTBidirectionalDouble(this, Math.toRadians(10.0));
      openedDoorHandleDistanceFromStart = new CRDTBidirectionalDouble(this, 0.12);
   }

   public void toMessage(DoorTraversalDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setLostGraspDetectionHandOpenAngle(lostGraspDetectionHandOpenAngle.toMessage());
      message.setOpenedDoorHandleDistanceFromStart(openedDoorHandleDistanceFromStart.toMessage());
   }

   public void fromMessage(DoorTraversalDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      lostGraspDetectionHandOpenAngle.fromMessage(message.getLostGraspDetectionHandOpenAngle());
      openedDoorHandleDistanceFromStart.fromMessage(message.getOpenedDoorHandleDistanceFromStart());
   }

   public CRDTBidirectionalDouble getLostGraspDetectionHandOpenAngle()
   {
      return lostGraspDetectionHandOpenAngle;
   }

   public CRDTBidirectionalDouble getOpenedDoorHandleDistanceFromStart()
   {
      return openedDoorHandleDistanceFromStart;
   }
}
