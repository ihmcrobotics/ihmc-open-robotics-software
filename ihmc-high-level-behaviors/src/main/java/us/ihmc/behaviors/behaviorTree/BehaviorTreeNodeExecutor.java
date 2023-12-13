package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                      D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeLayer<BehaviorTreeNodeExecutor<?, ?>, S, S, D>
{
   private final S state;
   private final List<BehaviorTreeNodeExecutor<?, ?>> children = new ArrayList<>();
   private transient BehaviorTreeNodeExecutor<?, ?> parent;

   /** For extending types. */
   public BehaviorTreeNodeExecutor(S state)
   {
      this.state = state;
   }

   /** For creating a basic node. */
   @SuppressWarnings("unchecked")
   public BehaviorTreeNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      D definition = (D) new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory);
      this.state = (S) new BehaviorTreeNodeState<D>(id, definition, crdtInfo);
   }

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
   public S getNextLowerLayer()
   {
      return getState();
   }

   @Override
   public D getDefinition()
   {
      return getState().getDefinition();
   }

   @Override
   public S getState()
   {
      return state;
   }
}
