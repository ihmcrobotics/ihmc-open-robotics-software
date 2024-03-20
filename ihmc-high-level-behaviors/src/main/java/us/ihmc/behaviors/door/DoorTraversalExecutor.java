package us.ihmc.behaviors.door;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;

   public DoorTraversalExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }
}
