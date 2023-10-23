package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class SakeHandCommandActionState extends BehaviorActionState
{
   private final SakeHandCommandActionDefinition definition;

   public SakeHandCommandActionState(long id, SakeHandCommandActionDefinition definition)
   {
      super(id, definition);

      this.definition = definition;
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   @Override
   public SakeHandCommandActionDefinition getDefinition()
   {
      return definition;
   }
}
