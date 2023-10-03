package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class WaitDurationActionState extends BehaviorActionState<WaitDurationActionDefinitionMessage>
{
   private final WaitDurationActionDefinition definition = new WaitDurationActionDefinition();

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}
