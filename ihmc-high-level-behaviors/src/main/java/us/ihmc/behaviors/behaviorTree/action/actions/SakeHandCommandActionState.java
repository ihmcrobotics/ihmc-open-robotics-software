package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class SakeHandCommandActionState extends ActionNodeState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new SakeHandCommandActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
