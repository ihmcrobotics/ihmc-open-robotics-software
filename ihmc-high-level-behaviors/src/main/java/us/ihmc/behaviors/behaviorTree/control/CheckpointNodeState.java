package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.CheckpointNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

public class CheckpointNodeState extends LeafNodeState<CheckpointNodeDefinition>
{
   public CheckpointNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new CheckpointNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(CheckpointNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(CheckpointNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      definition.fromMessage(message.getDefinition());
   }
}
