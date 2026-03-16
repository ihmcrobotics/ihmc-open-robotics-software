package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.CheckpointNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.LeafNodeDefinition;

public class CheckpointNodeDefinition extends LeafNodeDefinition
{
   public CheckpointNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(CheckpointNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(CheckpointNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
