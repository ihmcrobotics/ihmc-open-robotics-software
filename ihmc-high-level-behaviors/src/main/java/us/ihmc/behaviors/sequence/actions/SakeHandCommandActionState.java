package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;

public class SakeHandCommandActionState extends ActionNodeState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id)
   {
      super(id, new SakeHandCommandActionDefinition());
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      super.toMessage(message.getState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getState());
   }
}
