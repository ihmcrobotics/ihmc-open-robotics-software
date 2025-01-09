package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.function.Consumer;

/**
 * Common code for managing the tree between node implementations. i.e. RDX or Executor.
 *
 * @param <HLT> The generic type of this node: RDX or Executor
 */
public abstract class BehaviorTree<HLT extends BehaviorTreeNode<HLT, ? ,?>>
{
   private int numberOfNodes = 0;
   private final CRDTInfo crdtInfo;
   private final MutableLong nextID = new MutableLong(0);
   private final LatestTimestampModifiable rootReferenceModification;
   private final LatestTimestampModifiable dataModification;
   private final WorkspaceResourceDirectory saveFileDirectory;
   private final BehaviorTreeFileLoader<HLT> fileLoader;
   private final BehaviorTreeNodeBuilder<HLT> nodeBuilder;
   private final BehaviorTreeTopologyOperationQueue<HLT> topologyChangeQueue;

   public BehaviorTree(ROS2ActorDesignation actor,
                       ROS2PeerClockOffsetEstimator peerClockEstimator,
                       WorkspaceResourceDirectory saveFileDirectory,
                       BehaviorTreeNodeBuilder<HLT> nodeBuilder)
   {
      this.nodeBuilder = nodeBuilder;
      this.saveFileDirectory = saveFileDirectory;

      crdtInfo = new CRDTInfo(actor, peerClockEstimator);
      rootReferenceModification = new LatestTimestampModifiable(crdtInfo);
      rootReferenceModification.setModifierName("Root reference");
      dataModification = new LatestTimestampModifiable(crdtInfo);
      dataModification.setModifierName("Tree data");
      fileLoader = new BehaviorTreeFileLoader<>(this, nodeBuilder, saveFileDirectory);
      topologyChangeQueue = new BehaviorTreeTopologyOperationQueue<>(this);
   }

   /** Used only when modifying tree topology. */
   private void update()
   {
      numberOfNodes = 0;
      update(getRootNode());
   }

   private void update(HLT node)
   {
      if (node != null)
      {
         ++numberOfNodes;

         for (HLT child : node.getChildren())
         {
            update(child);
         }
      }
   }

   /**
    * Use to safely modify the tree, ensuring it's updated afterwards.
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

   public void deleteRootNode()
   {
      modifyTreeTopology(BehaviorTreeTopologyOperationQueue::queueDestroyEntireTreeModify);
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

   public abstract void setRootNode(HLT rootNode);

   public abstract HLT getRootNode();

   public int getNumberOfNodes()
   {
      return numberOfNodes;
   }

   public CRDTInfo getCRDTInfo()
   {
      return crdtInfo;
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

   public LatestTimestampModifiable getRootReferenceModification()
   {
      return rootReferenceModification;
   }

   public LatestTimestampModifiable getDataModification()
   {
      return dataModification;
   }

   public WorkspaceResourceDirectory getSaveFileDirectory()
   {
      return saveFileDirectory;
   }

   public BehaviorTreeFileLoader<HLT> getFileLoader()
   {
      return fileLoader;
   }

   public BehaviorTreeNodeBuilder<HLT> getNodeBuilder()
   {
      return nodeBuilder;
   }

   public BehaviorTreeTopologyOperationQueue<HLT> getTopologyChangeQueue()
   {
      return topologyChangeQueue;
   }
}
