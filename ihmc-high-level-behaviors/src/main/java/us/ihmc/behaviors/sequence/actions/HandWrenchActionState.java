package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class HandWrenchActionState extends BehaviorActionState
{
   private final HandWrenchActionDefinition definition;

   public HandWrenchActionState(long id, HandWrenchActionDefinition definition)
   {
      super(id, definition);

      this.definition = definition;
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
