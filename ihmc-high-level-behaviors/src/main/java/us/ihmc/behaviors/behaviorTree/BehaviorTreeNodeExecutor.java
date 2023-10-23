package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;
import java.util.List;

public abstract class BehaviorTreeNodeExecutor<E extends BehaviorTreeNodeExecutor<E, S, D>,
                                               S extends BehaviorTreeNodeState<S, D>,
                                               D extends BehaviorTreeNodeDefinition<D>>
      implements BehaviorTreeNodeExtension<E, S>,
                 BehaviorTreeNodeStateSupplier<S, D>
{
   private final List<E> children = new ArrayList<>();

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

   public void tick()
   {
      getState().setIsActive(true);
   }

   @Override
   public void destroy()
   {

   }

   public List<E> getChildren()
   {
      return children;
   }

   @Override
   public S getExtendedNode()
   {
      return getState();
   }
}
