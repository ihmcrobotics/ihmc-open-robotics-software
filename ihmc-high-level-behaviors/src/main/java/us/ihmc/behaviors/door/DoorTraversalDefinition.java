package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTUnidirectionalDouble lostGraspDetectionHandOpenAngle;
   private final CRDTUnidirectionalDouble openedDoorHandleDistanceFromStart;

   public DoorTraversalDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      lostGraspDetectionHandOpenAngle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, Math.toRadians(10.0));
      openedDoorHandleDistanceFromStart = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
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

   public CRDTUnidirectionalDouble getLostGraspDetectionHandOpenAngle()
   {
      return lostGraspDetectionHandOpenAngle;
   }

   public CRDTUnidirectionalDouble getOpenedDoorHandleDistanceFromStart()
   {
      return openedDoorHandleDistanceFromStart;
   }
}
