package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class HandWrenchActionState extends BehaviorActionState<HandWrenchActionDefinitionMessage>
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
