package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class WaitDurationActionState extends ActionNodeState<WaitDurationActionDefinition>
{
   public WaitDurationActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new WaitDurationActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
