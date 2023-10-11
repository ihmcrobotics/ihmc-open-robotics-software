package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class HandWrenchActionState extends BehaviorActionState
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();

   public void toMessage(HandWrenchActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
