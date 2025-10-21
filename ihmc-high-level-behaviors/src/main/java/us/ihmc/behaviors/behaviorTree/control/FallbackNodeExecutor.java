package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

/**
 * If the first concurrent leaf group of a fallback node succeeds, the rest of the children are skipped.
 * If it fails, the rest of the children are executed.
 */
public class FallbackNodeExecutor extends BehaviorTreeNodeExecutor<FallbackNodeState, FallbackNodeDefinition>
{
   private final List<LeafNodeExecutor<?, ?>> leafChildren = new ArrayList<>();

   // TODO: Add these to state & add UI elements
   private final List<LeafNodeExecutor<?, ?>> tryLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> fallbackLeaves = new ArrayList<>();

   public FallbackNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new FallbackNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void update()
   {
      super.update();

      leafChildren.clear();
      tryLeaves.clear();
      fallbackLeaves.clear();

      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
      {
         if (child instanceof LeafNodeExecutor<?, ?> leafNode)
         {
            leafChildren.add(leafNode);
         }
      }

      if (!leafChildren.isEmpty())
      {
         int firstLeafIndex = leafChildren.get(0).getState().getLeafIndex();

         for (LeafNodeExecutor<?, ?> child : leafChildren)
         {
            if (child.getState().getExecuteAfterLeafIndex() < firstLeafIndex)
            {
               tryLeaves.add(child);
            }
            else
            {
               fallbackLeaves.add(child);
            }
         }
      }
   }

   public List<LeafNodeExecutor<?, ?>> getLeafChildren()
   {
      return leafChildren;
   }

   public List<LeafNodeExecutor<?, ?>> getTryLeaves()
   {
      return tryLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getFallbackLeaves()
   {
      return fallbackLeaves;
   }
}
