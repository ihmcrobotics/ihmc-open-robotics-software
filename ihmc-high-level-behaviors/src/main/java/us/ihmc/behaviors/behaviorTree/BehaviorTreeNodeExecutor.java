package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;
import java.util.List;

public abstract class BehaviorTreeNodeExecutor implements BehaviorTreeNodeStateSupplier
{
   private final List<BehaviorTreeNodeExecutor> children = new ArrayList<>();

   /**
    * A method that should be called before each {@link #tick}
    * in order for nodes to know when they are no longer being selected.
    */
   public void clock()
   {
      getState().setIsActive(false);

      for (BehaviorTreeNodeExecutor child : children)
      {
         child.clock();
      }
   }

   public BehaviorTreeNodeStatus tick()
   {
      getState().setIsActive(true);
      getState().setStatus(tickInternal());
      return getState().getStatus();
   }

   // TODO: Is this a good convention doing the *Internal thing?
   //   or would it be better to call super.tick or something somehow
   protected BehaviorTreeNodeStatus tickInternal()
   {
      return getState().getStatus();
   }

   public List<BehaviorTreeNodeExecutor> getChildren()
   {
      return children;
   }
}
