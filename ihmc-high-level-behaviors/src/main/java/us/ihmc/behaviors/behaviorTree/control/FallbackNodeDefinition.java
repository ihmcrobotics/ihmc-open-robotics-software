package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.FallbackNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;

public class FallbackNodeDefinition extends BehaviorTreeNodeDefinition
{
   public FallbackNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(FallbackNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(FallbackNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
