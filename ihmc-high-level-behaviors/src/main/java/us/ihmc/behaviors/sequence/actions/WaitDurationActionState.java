package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;

public class WaitDurationActionState extends ActionNodeState<WaitDurationActionDefinition>
{
   public WaitDurationActionState(long id)
   {
      super(id, new WaitDurationActionDefinition());
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      super.toMessage(message.getState());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      super.fromMessage(message.getState());
   }
}
