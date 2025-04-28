package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTBidirectionalDouble lostGraspDetectionHandOpenAngle;
   private final CRDTBidirectionalDouble openedDoorHandleDistanceFromStart;

   public DoorTraversalDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

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
