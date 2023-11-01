package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class ArmJointAnglesActionState extends ActionNodeState<ArmJointAnglesActionDefinition>
{
   public ArmJointAnglesActionState(long id, ROS2ActorDesignation actorDesignation)
   {
      super(id, new ArmJointAnglesActionDefinition(), actorDesignation);
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
