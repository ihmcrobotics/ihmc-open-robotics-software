package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.WaitActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class WaitActionState extends ActionNodeState<WaitActionDefinition>
{
   public WaitActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new WaitActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(WaitActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(WaitActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
