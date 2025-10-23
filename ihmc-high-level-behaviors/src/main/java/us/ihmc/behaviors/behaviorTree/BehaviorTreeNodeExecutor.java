package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The base class for an executor node, which is the type that solely exists
 * on board the robot and controls the robot.
 *
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public class BehaviorTreeNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                      D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNode<BehaviorTreeNodeExecutor<?, ?>, S, D>
{
   /** Convenient accessor to the state to keep the code clean, available to all inheriting classes. */
   protected final S state;
   /** Convenient accessor to the definition to keep the code clean, available to all inheriting classes. */
   protected final D definition;
   private final List<BehaviorTreeNodeExecutor<?, ?>> children = new ArrayList<>();
   private transient BehaviorTreeNodeExecutor<?, ?> parent;

   /** For extending types. */
   public BehaviorTreeNodeExecutor(S state)
   {
      this.state = state;
      definition = state.getDefinition();
   }

   /** For creating a basic node. */ // TODO: Should not exist???
   @SuppressWarnings("unchecked")
   public BehaviorTreeNodeExecutor(long id, CRDTInfo crdtInfo)
   {
      definition = (D) new BehaviorTreeNodeDefinition(crdtInfo);
      this.state = (S) new BehaviorTreeNodeState<D>(id, definition);
   }

   /**
    * A method that should be called before each {@link #tick}
    * in order for nodes to know when they are no longer being selected.
    */
   public void clock()
   {
      state.setIsActive(false);

      for (BehaviorTreeNodeExecutor<?, ?> child : children)
      {
         child.clock();
      }
   }

   public void tick()
   {
      state.setIsActive(true);
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
   public D getDefinition()
   {
      return definition;
   }

   @Override
   public S getState()
   {
      return state;
   }
}
