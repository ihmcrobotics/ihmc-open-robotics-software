package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.CheckPointNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

public class CheckPointNodeState extends LeafNodeState<CheckPointNodeDefinition>
{
   public CheckPointNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new CheckPointNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(CheckPointNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(CheckPointNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      definition.fromMessage(message.getDefinition());
   }
}
