package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.AbilityHandActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class AbilityHandActionState extends ActionNodeState<AbilityHandActionDefinition>
{
   public AbilityHandActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new AbilityHandActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(AbilityHandActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(AbilityHandActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
