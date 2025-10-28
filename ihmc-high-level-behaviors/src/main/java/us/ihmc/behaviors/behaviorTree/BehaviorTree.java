package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeMessageTools;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
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
 * @param <R> The type of root node: RDX or Executor
 * @param <T> The generic type of this node: RDX or Executor
 */
public abstract class BehaviorTree<R extends BehaviorTreeRootNode<T>, T extends BehaviorTreeNode<T, ?, ?>>
{
   protected int numberOfNodes = 0;
   protected final CRDTInfo crdtInfo;
   protected final WorkspaceResourceDirectory saveFileDirectory;
   protected final DRCRobotModel robotModel;
   protected final ROS2SyncedRobotModel syncedRobot;
   private final MutableLong nextID = new MutableLong(0);
   private final LatestTimestampModifiable rootReferenceModification;
   private final LatestTimestampModifiable dataModification;
   private final BehaviorTreeFileLoader<T> fileLoader;
   private final BehaviorTreeNodeBuilder<T> nodeBuilder;
   private final BehaviorTreeTopologyOperationQueue<T> topologyChangeQueue;
   protected R rootNode;
   private BehaviorTreeSceneState scene;

   public BehaviorTree(ROS2SyncedRobotModel syncedRobot,
                       ROS2ActorDesignation actor,
                       ROS2PeerClockOffsetEstimator peerClockEstimator,
                       WorkspaceResourceDirectory saveFileDirectory,
                       BehaviorTreeNodeBuilder<T> nodeBuilder)
   {
      this.robotModel = syncedRobot.getRobotModel();
      this.syncedRobot = syncedRobot;
      this.nodeBuilder = nodeBuilder;
      this.saveFileDirectory = saveFileDirectory;

      crdtInfo = new CRDTInfo(actor, peerClockEstimator);
      rootReferenceModification = new LatestTimestampModifiable(crdtInfo);
      rootReferenceModification.setModifierName("Root reference");
      dataModification = new LatestTimestampModifiable(crdtInfo);
      dataModification.setModifierName("Tree data");
      fileLoader = new BehaviorTreeFileLoader<>(this, nodeBuilder, saveFileDirectory);
      topologyChangeQueue = new BehaviorTreeTopologyOperationQueue<>((BehaviorTree) this);
   }

   /** Used only when modifying tree topology. */
   private void update()
   {
      numberOfNodes = 0;
      update(getRootNode());
   }

   private void update(TreeNode<?> node)
   {
      if (node != null)
      {
         ++numberOfNodes;

         for (TreeNode<?> child : node.getChildren())
         {
            update(child);
         }
      }
   }

   /**
    * Use to safely modify the tree, ensuring it's updated afterwards.
    */
   public void modifyTreeTopology(Consumer<BehaviorTreeTopologyOperationQueue<T>> modifier)
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
      scene.toMessage(message.getScene());
      ROS2BehaviorTreeMessageTools.clearLists(message);
      if (rootNode != null)
         toMessage(message, rootNode.getState());
   }

   private void toMessage(BehaviorTreeStateMessage message, BehaviorTreeNodeState<?> behaviorTreeNode)
   {
      ROS2BehaviorTreeMessageTools.packMessage(crdtInfo, behaviorTreeNode, message);
      for (BehaviorTreeNodeState<?> child : behaviorTreeNode.getChildren())
      {
         toMessage(message, child);
      }
   }

   public void fromMessage(BehaviorTreeStateMessage message)
   {
      rootReferenceModification.fromMessage(message.getLatestModificationToRootReference());
      dataModification.fromMessage(message.getLatestModificationToData());
      scene.fromMessage(message.getScene());

      if (dataModification.isModificationIncoming())
         nextID.setValue(message.getNextId());
   }

   public void setScene(BehaviorTreeSceneState scene)
   {
      this.scene = scene;
   }

   public void setRootNode(R rootNode)
   {
      this.rootNode = rootNode;
   }

   public R getRootNode()
   {
      return rootNode;
   }

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

   public BehaviorTreeFileLoader<T> getFileLoader()
   {
      return fileLoader;
   }

   public BehaviorTreeNodeBuilder<T> getNodeBuilder()
   {
      return nodeBuilder;
   }

   public BehaviorTreeTopologyOperationQueue<T> getTopologyChangeQueue()
   {
      return topologyChangeQueue;
   }
}
