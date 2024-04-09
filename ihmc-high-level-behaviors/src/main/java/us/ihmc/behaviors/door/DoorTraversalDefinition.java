package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTUnidirectionalDouble minimumHandOpenAngleDegrees;

   public DoorTraversalDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      minimumHandOpenAngleDegrees = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 10.0);
   }

   public void toMessage(DoorTraversalDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setMinimumHandOpenAngle(minimumHandOpenAngleDegrees.toMessage());
   }

   public void fromMessage(DoorTraversalDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      minimumHandOpenAngleDegrees.fromMessage(message.getMinimumHandOpenAngle());
   }

   public double getMinimumHandOpenAngle()
   {
      return minimumHandOpenAngleDegrees.getValue();
   }

   public void setMinimumHandOpenAngle(double handOpenAngle)
   {
      this.minimumHandOpenAngleDegrees.setValue(handOpenAngle);
   }
}
