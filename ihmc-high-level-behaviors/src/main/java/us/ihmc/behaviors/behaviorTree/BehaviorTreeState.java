package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.RequestConfirmFreezable;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState extends RequestConfirmFreezable
{
   private final MutableLong nextID = new MutableLong(0);
   private final BehaviorTreeTopologyOperationQueue topologyChangeQueue = new BehaviorTreeTopologyOperationQueue();
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

   /**
    * Convenience method.
    */
   public void modifyTreeTopology(Consumer<BehaviorTreeTopologyOperationQueue> modifier)
   {
      modifier.accept(topologyChangeQueue);
      modifyTreeTopology();
   }

   /**
    * Use with {@link #getTopologyChangeQueue()}.
    */
   public void modifyTreeTopology()
   {
      boolean atLeastOnePerformed = topologyChangeQueue.performAllQueuedOperations();

      if (atLeastOnePerformed)
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

   public BehaviorTreeTopologyOperationQueue getTopologyChangeQueue()
   {
      return topologyChangeQueue;
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
