package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class SakeHandCommandActionState extends BehaviorActionState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id)
   {
      super(id, new SakeHandCommandActionDefinition());
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }
}
