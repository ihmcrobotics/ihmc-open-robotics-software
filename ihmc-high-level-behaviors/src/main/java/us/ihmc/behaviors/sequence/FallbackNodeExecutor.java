package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

/**
 * If the first concurrent action group of a fallback node succeeds, the rest of the children are skipped.
 * If it fails, the rest of the children are executed.
 */
public class FallbackNodeExecutor extends BehaviorTreeNodeExecutor<FallbackNodeState, FallbackNodeDefinition>
{
   private final FallbackNodeState state;
   private final FallbackNodeDefinition definition;
   private final List<ActionNodeExecutor<?, ?>> actionChildren = new ArrayList<>();

   // TODO: Add these to state & add UI elements
   private final List<ActionNodeExecutor<?, ?>> tryActions = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> fallbackActions = new ArrayList<>();

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
      tryActions.clear();
      fallbackActions.clear();

      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {
            actionChildren.add(actionNode);
         }
      }

      if (!actionChildren.isEmpty())
      {
         int firstActionIndex = actionChildren.get(0).getState().getActionIndex();

         for (ActionNodeExecutor<?, ?> child : actionChildren)
         {
            if (child.getState().calculateExecuteAfterActionIndex() < firstActionIndex)
            {
               tryActions.add(child);
            }
            else
            {
               fallbackActions.add(child);
            }
         }
      }
   }

   public List<ActionNodeExecutor<?, ?>> getActionChildren()
   {
      return actionChildren;
   }

   public List<ActionNodeExecutor<?, ?>> getTryActions()
   {
      return tryActions;
   }

   public List<ActionNodeExecutor<?, ?>> getFallbackActions()
   {
      return fallbackActions;
   }
}
