package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public abstract class BehaviorTreeNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                               D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeExtension<BehaviorTreeNodeExecutor<?, ?>, S, S, D>
{
   private final List<BehaviorTreeNodeExecutor<?, ?>> children = new ArrayList<>();
   private transient BehaviorTreeNodeExecutor<?, ?> parent;

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
      LogTools.info("Destroying node: {}:{}", getState().getDefinition().getDescription(), getState().getID());
      getState().destroy();
   }

   public List<BehaviorTreeNodeExecutor<?, ?>> getChildren()
   {
      return children;
   }

   @Override
   public void setParent(@Nullable BehaviorTreeNodeExecutor<?, ?> parent)
   {
      this.parent = parent;
   }

   @Nullable
   @Override
   public BehaviorTreeNodeExecutor<?, ?> getParent()
   {
      return parent;
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
