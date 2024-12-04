package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

/**
 * If the first child action of a fallback node succeeds, the rest of the children are skipped.
 * If it fails, the rest of the children are executed.
 * Afterwards, the parent sequence proceeds.
 */
public class FallbackNodeExecutor extends BehaviorTreeNodeExecutor<FallbackNodeState, FallbackNodeDefinition>
{
   private final FallbackNodeState state;
   private final FallbackNodeDefinition definition;
   private final List<ActionNodeExecutor<?, ?>> actionChildren = new ArrayList<>();

   public FallbackNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new FallbackNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      actionChildren.clear();
      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {
            actionChildren.add(actionNode);
         }
      }
   }

   public List<ActionNodeExecutor<?, ?>> getActionChildren()
   {
      return actionChildren;
   }
}
