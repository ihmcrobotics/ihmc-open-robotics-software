package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModification;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.Confirmable;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState extends Confirmable
{
   private final MutableLong nextID = new MutableLong(0);
   private final Queue<BehaviorTreeModification> queuedModifications = new LinkedList<>();
   private final BehaviorTreeNodeStateBuilder nodeStateBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final Supplier<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSupplier;
   private int numberOfNodes = 0;
   private int numberOfFrozenNodes = 0;

   public BehaviorTreeState(BehaviorTreeNodeStateBuilder nodeStateBuilder,
                            BehaviorTreeExtensionSubtreeRebuilder treeRebuilder,
                            Supplier<BehaviorTreeNodeExtension<?, ?, ?, ?>> rootNodeSupplier,
                            CRDTInfo crdtInfo)
   {
      super(crdtInfo);

      this.nodeStateBuilder = nodeStateBuilder;
      this.treeRebuilder = treeRebuilder;
      this.rootNodeSupplier = rootNodeSupplier;
   }

   public void update()
   {
      numberOfNodes = 0;
      numberOfFrozenNodes = 0;
      update(rootNodeSupplier.get());
   }

   private void update(BehaviorTreeNodeExtension<?, ?, ?, ?> node)
   {
      if (node != null)
      {
         ++numberOfNodes;
         if (node.getState().isFrozen())
            ++numberOfFrozenNodes;

         for (Object child : node.getChildren())
         {
            update((BehaviorTreeNodeExtension<?, ?, ?, ?>) child);
         }
      }
   }

   public void modifyTree(Consumer<BehaviorTreeModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         BehaviorTreeModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   public void toMessage(BehaviorTreeStateMessage message)
   {
      message.setNextId(nextID.longValue());
      toMessage(message.getConfirmableRequest());
   }

   public void fromMessage(BehaviorTreeStateMessage message)
   {
      fromMessage(message.getConfirmableRequest());

      if (!isFrozen())
         nextID.setValue(message.getNextId());
   }

   public long getAndIncrementNextID()
   {
      freeze();
      return nextID.getAndIncrement();
   }

   public long getNextID()
   {
      return nextID.longValue();
   }

   public BehaviorTreeNodeExtension<?, ?, ?, ?> getRootNode()
   {
      return rootNodeSupplier.get();
   }

   public BehaviorTreeNodeStateBuilder getNodeStateBuilder()
   {
      return nodeStateBuilder;
   }

   public BehaviorTreeExtensionSubtreeRebuilder getTreeRebuilder()
   {
      return treeRebuilder;
   }

   public int getNumberOfFrozenNodes()
   {
      return numberOfFrozenNodes;
   }

   public int getNumberOfNodes()
   {
      return numberOfNodes;
   }
}
