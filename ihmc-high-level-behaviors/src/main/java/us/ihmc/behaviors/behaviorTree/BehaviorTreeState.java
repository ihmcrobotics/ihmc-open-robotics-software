package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModification;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModificationQueue;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState
{
   private final MutableLong nextID = new MutableLong(0);
   private final Queue<BehaviorTreeStateModification> queuedModifications = new LinkedList<>();
   private final BehaviorTreeNodeTypeClassSupplier nodeTypeClassSupplier;
   private final BehaviorTreeNodeStateBuilder nodeStateBuilder;

   private BehaviorTreeNodeState rootNode;

   public BehaviorTreeState(BehaviorTreeNodeTypeClassSupplier nodeTypeClassSupplier,
                            BehaviorTreeNodeStateBuilder nodeStateBuilder)
   {
      this.nodeTypeClassSupplier = nodeTypeClassSupplier;
      this.nodeStateBuilder = nodeStateBuilder;
   }

   public void update()
   {

   }

   public void modifyTree(Consumer<BehaviorTreeStateModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         BehaviorTreeStateModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   public MutableLong getNextID()
   {
      return nextID;
   }

   public BehaviorTreeNodeState getRootNode()
   {
      return rootNode;
   }

   public BehaviorTreeNodeTypeClassSupplier getNodeTypeClassSupplier()
   {
      return nodeTypeClassSupplier;
   }

   public BehaviorTreeNodeStateBuilder getNodeStateBuilder()
   {
      return nodeStateBuilder;
   }
}
