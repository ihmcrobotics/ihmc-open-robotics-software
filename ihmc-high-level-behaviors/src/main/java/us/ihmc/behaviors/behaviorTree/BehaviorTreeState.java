package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 *
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 */
public class BehaviorTreeState<HLT extends BehaviorTreeNodeHighLayer<HLT, ? ,?>>
{
   private final CRDTInfo crdtInfo;
   private final LatestTimestampModifiable rootReferenceModification;
   private final LatestTimestampModifiable dataModification;
   private final MutableLong nextID = new MutableLong(0);
   private final BehaviorTreeTopologyOperationQueue<HLT> topologyChangeQueue = new BehaviorTreeTopologyOperationQueue<>();
   private final BehaviorTreeNodeStateBuilder<HLT> nodeStateBuilder;
   private final Supplier<HLT> rootNodeSupplier;
   private final WorkspaceResourceDirectory saveFileDirectory;
   private int numberOfNodes = 0;

   public BehaviorTreeState(BehaviorTreeNodeStateBuilder<HLT> nodeStateBuilder,
                            Supplier<HLT> rootNodeSupplier,
                            CRDTInfo crdtInfo,
                            WorkspaceResourceDirectory saveFileDirectory)
   {
      this.crdtInfo = crdtInfo;
      this.rootReferenceModification = new LatestTimestampModifiable(crdtInfo);
      this.dataModification = new LatestTimestampModifiable(crdtInfo);
      this.nodeStateBuilder = nodeStateBuilder;
      this.rootNodeSupplier = rootNodeSupplier;
      this.saveFileDirectory = saveFileDirectory;
   }

   /** Used only when modifying tree topology. */
   private void update()
   {
      numberOfNodes = 0;
      update(rootNodeSupplier.get());
   }

   private void update(BehaviorTreeNode<?> node)
   {
      if (node != null)
      {
         ++numberOfNodes;

         for (BehaviorTreeNode<?> child : node.getChildren())
         {
            update(child);
         }
      }
   }

   /**
    * Convenience method.
    */
   public void modifyTreeTopology(Consumer<BehaviorTreeTopologyOperationQueue<HLT>> modifier)
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
      message.setSequenceId(crdtInfo.getUpdateNumber());
      message.setNextId(nextID.longValue());
      rootReferenceModification.toMessage(message.getLatestModificationToRootReference());
      dataModification.toMessage(message.getLatestModificationToData());
   }

   public void fromMessage(BehaviorTreeStateMessage message)
   {
      rootReferenceModification.fromMessage(message.getLatestModificationToRootReference());
      dataModification.fromMessage(message.getLatestModificationToData());

      if (dataModification.isModificationIncoming())
         nextID.setValue(message.getNextId());
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
   }

   public LatestTimestampModifiable getRootReferenceModification()
   {
      return rootReferenceModification;
   }

   public LatestTimestampModifiable getDataModification()
   {
      return dataModification;
   }

   public long getAndIncrementNextID()
   {
      dataModification.modify();
      return nextID.getAndIncrement();
   }

   public long getNextID()
   {
      return nextID.longValue();
   }

   public HLT getRootNode()
   {
      return rootNodeSupplier.get();
   }

   public BehaviorTreeNodeStateBuilder<HLT> getNodeStateBuilder()
   {
      return nodeStateBuilder;
   }

   public WorkspaceResourceDirectory getSaveFileDirectory()
   {
      return saveFileDirectory;
   }

   public BehaviorTreeTopologyOperationQueue<HLT> getTopologyChangeQueue()
   {
      return topologyChangeQueue;
   }

   public int getNumberOfNodes()
   {
      return numberOfNodes;
   }
}
