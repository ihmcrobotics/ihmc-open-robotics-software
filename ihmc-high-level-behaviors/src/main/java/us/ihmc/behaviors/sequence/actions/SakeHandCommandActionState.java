package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class SakeHandCommandActionState extends BehaviorActionState<SakeHandCommandActionDefinitionMessage>
{
   private final SakeHandCommandActionDefinition definition = new SakeHandCommandActionDefinition();

   @Override
   public SakeHandCommandActionDefinition getDefinition()
   {
      return definition;
   }
}
