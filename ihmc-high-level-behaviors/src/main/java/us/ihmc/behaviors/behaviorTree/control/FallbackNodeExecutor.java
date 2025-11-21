package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;

/**
 * If the first concurrent leaf group of a fallback node succeeds, the rest of the children are skipped.
 * If it fails, the rest of the children are executed.
 *
 * TODO: Support nested fallbacks
 */
public class FallbackNodeExecutor extends BehaviorTreeNodeExecutor<FallbackNodeState, FallbackNodeDefinition>
{
   private final List<LeafNodeExecutor<?, ?>> childrenLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> tryLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> catchLeaves = new ArrayList<>();

   public FallbackNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new FallbackNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      childrenLeaves.clear();
      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
         if (child instanceof LeafNodeExecutor<?, ?> leafNode)
            childrenLeaves.add(leafNode);

      tryLeaves.clear();
      catchLeaves.clear();
      if (!childrenLeaves.isEmpty())
      {
         int firstLeafIndex = childrenLeaves.get(0).getState().getLeafIndex();

         for (LeafNodeExecutor<?, ?> child : childrenLeaves)
            if (child.getState().getExecuteAfterLeafIndex() < firstLeafIndex)
               tryLeaves.add(child);
            else
               catchLeaves.add(child);
      }
   }

   public boolean tryLeafIsBlocking(LeafNodeExecutor<?, ?> leaf)
   {
      if (catchLeaves.contains(leaf))
         for (LeafNodeExecutor<?, ?> tryLeaf : tryLeaves)
            if (tryLeaf.getState().getIsExecuting())
               return true;

      return false;
   }

   /** Call only after a check to ensure leaf is one of the children. */
   public boolean leafCeasedExecution(LeafNodeExecutor<?, ?> leaf)
   {
      if (catchLeaves.isEmpty())
      {
         LogTools.error("There are no catch leaves. Try logic is disabled.");
      }
      else if (tryLeaves.contains(leaf))
      {
         if (leaf.getState().getFailed())
         {
            LeafNodeExecutor<?, ?> firstCatchLeaf = catchLeaves.get(0);
            LogTools.warn("Try leaf failed, skipping to first catch leaf: %s".formatted(firstCatchLeaf.getDefinition().getName()));
            rootNode.getState().setExecutionNextIndex(firstCatchLeaf.getState().getLeafIndex()); // Minus one because gets incremented after
         }
         else if (tryLeaves.indexOf(leaf) == tryLeaves.size() - 1) // Last try leaf succeeded
         {
            String gotoNodeName = "End of sequence";
            int indexAfterLastCatchLeaf = catchLeaves.get(catchLeaves.size() - 1).getState().getLeafIndex() + 1;
            if (rootNode.getOrderedLeaves().size() > indexAfterLastCatchLeaf)
               gotoNodeName = rootNode.getOrderedLeaves().get(indexAfterLastCatchLeaf).getDefinition().getName();

            LogTools.info("Last try leaf successful, skipping fallback leaves to: %s".formatted(gotoNodeName));
            rootNode.getState().setExecutionNextIndex(indexAfterLastCatchLeaf); // Minus one because gets incremented after
         }

         return true;
      }

      return false;
   }

   public List<LeafNodeExecutor<?, ?>> getChildrenLeaves()
   {
      return childrenLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getTryLeaves()
   {
      return tryLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getCatchLeaves()
   {
      return catchLeaves;
   }
}
