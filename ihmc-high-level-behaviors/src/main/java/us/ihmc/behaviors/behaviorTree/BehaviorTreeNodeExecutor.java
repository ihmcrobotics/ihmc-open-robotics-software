package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;
import java.util.List;

public abstract class BehaviorTreeNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                               D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeExtension<BehaviorTreeNodeExecutor<?, ?>, S, S, D>
{
   private final List<BehaviorTreeNodeExecutor<?, ?>> children = new ArrayList<>();

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

   public List<BehaviorTreeNodeExecutor<?, ?>> getChildren()
   {
      return children;
   }

   @Override
   public S getExtendedNode()
   {
      return getState();
   }

   @Override
   public D getDefinition()
   {
      return getState().getDefinition();
   }
}
