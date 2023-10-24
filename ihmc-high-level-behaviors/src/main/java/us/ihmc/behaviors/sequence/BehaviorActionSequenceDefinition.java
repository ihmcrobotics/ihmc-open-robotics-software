package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

// FIXME: Duplicate with BehaviorActionDefinition
public class BehaviorActionSequenceDefinition extends BehaviorTreeNodeDefinition
{
   // Seems to be nothing special here so far TODO Does that mean we delete it?

   public void toMessage(BehaviorActionSequenceDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(BehaviorActionSequenceDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
