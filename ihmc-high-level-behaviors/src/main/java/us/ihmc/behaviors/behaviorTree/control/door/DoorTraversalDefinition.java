package us.ihmc.behaviors.behaviorTree.control.door;

import behavior_msgs.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   public DoorTraversalDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
   }

   public void toMessage(DoorTraversalDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(DoorTraversalDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
