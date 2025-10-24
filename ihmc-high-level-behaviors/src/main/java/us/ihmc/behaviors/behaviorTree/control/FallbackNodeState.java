package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.FallbackNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;

public class FallbackNodeState extends BehaviorTreeNodeState<FallbackNodeDefinition>
{
   public FallbackNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new FallbackNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(FallbackNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FallbackNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
