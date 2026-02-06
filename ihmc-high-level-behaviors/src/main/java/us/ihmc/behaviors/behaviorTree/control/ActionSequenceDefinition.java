package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.ActionSequenceDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;

public class ActionSequenceDefinition extends BehaviorTreeNodeDefinition
{
   // Seems to be nothing special here so far TODO Does that mean we delete it?

   public ActionSequenceDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(ActionSequenceDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(ActionSequenceDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
