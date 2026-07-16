package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.HandWrenchActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class HandWrenchActionState extends ActionNodeState<HandWrenchActionDefinition>
{
   public HandWrenchActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new HandWrenchActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
