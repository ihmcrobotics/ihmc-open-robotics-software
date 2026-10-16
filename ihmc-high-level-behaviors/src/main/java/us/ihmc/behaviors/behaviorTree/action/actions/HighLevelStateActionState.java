package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.HighLevelStateActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class HighLevelStateActionState extends ActionNodeState<HighLevelStateActionDefinition>
{
   public HighLevelStateActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new HighLevelStateActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(HighLevelStateActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HighLevelStateActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
