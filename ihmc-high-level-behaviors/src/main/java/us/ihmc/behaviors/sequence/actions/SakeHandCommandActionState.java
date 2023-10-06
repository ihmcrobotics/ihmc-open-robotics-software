package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class SakeHandCommandActionState extends BehaviorActionState
{
   private final SakeHandCommandActionDefinition definition = new SakeHandCommandActionDefinition();

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public SakeHandCommandActionDefinition getDefinition()
   {
      return definition;
   }
}
