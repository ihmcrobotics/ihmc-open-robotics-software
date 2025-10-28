package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.CheckPointNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.LeafNodeDefinition;

public class CheckPointNodeDefinition extends LeafNodeDefinition
{
   public CheckPointNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(CheckPointNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(CheckPointNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
