package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.EZGripperActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class EZGripperActionState extends ActionNodeState<EZGripperActionDefinition>
{
   public EZGripperActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new EZGripperActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(EZGripperActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(EZGripperActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
