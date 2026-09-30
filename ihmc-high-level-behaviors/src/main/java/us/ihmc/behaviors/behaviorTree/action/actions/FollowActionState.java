package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.FollowActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class FollowActionState extends ActionNodeState<FollowActionDefinition>
{
   public FollowActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new FollowActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(FollowActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FollowActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
