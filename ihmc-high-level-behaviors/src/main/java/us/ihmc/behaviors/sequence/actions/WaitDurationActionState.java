package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class WaitDurationActionState extends BehaviorActionState
{
   private final WaitDurationActionDefinition definition;

   public WaitDurationActionState(long id, WaitDurationActionDefinition definition)
   {
      super(id, definition);

      this.definition = definition;
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}
