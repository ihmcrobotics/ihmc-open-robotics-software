package us.ihmc.behaviors.behaviorTree.control.door;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;

public class DoorTraversalState extends BehaviorTreeNodeState<DoorTraversalDefinition>
{
   public DoorTraversalState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new DoorTraversalDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(DoorTraversalStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(DoorTraversalStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
