package us.ihmc.behaviors.behaviorTree.control.door;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   public DoorTraversalExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }
}
