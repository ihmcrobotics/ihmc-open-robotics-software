package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;

public class AI2RNodeDefinition extends BehaviorTreeNodeDefinition
{
   public AI2RNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(AI2RNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(AI2RNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
