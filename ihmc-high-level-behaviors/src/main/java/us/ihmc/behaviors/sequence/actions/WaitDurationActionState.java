package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class WaitDurationActionState extends ActionNodeState<WaitDurationActionDefinition>
{
   public WaitDurationActionState(long id, ROS2ActorDesignation actorDesignation)
   {
      super(id, new WaitDurationActionDefinition(), actorDesignation);
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
