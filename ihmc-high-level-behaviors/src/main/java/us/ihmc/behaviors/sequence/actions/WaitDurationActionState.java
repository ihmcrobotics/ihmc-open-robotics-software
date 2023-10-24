package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class WaitDurationActionState extends BehaviorActionState<WaitDurationActionDefinition>
{
   public WaitDurationActionState(long id)
   {
      super(id, new WaitDurationActionDefinition());
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }
}
