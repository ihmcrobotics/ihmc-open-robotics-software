package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.MimicActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class MimicActionState extends ActionNodeState<MimicActionDefinition>
{
   public MimicActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new MimicActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(MimicActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(MimicActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
