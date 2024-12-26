package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeExecutor
{
   private final CRDTInfo crdtInfo;
   private final BehaviorTreeExecutorNodeBuilder nodeBuilder;
   private final BehaviorTreeState state;
   private BehaviorTreeRootNodeExecutor rootNode;
   private final BehaviorTreeFileLoader<BehaviorTreeNodeExecutor<?, ?>> fileLoader;
   private final WorkspaceResourceDirectory saveFileDirectory = new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees");

   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2PeerClockOffsetEstimator peerClockEstimator,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               SceneGraph sceneGraph,
                               DetectionManager detectionManager,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);
      nodeBuilder = new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot, referenceFrameLibrary, sceneGraph, detectionManager);

      state = new BehaviorTreeState(nodeBuilder, this::getRootNode, crdtInfo, null);
      fileLoader = new BehaviorTreeFileLoader<>(state, nodeBuilder, saveFileDirectory);
   }

   public void update()
   {
      if (rootNode != null)
      {
         rootNode.clock();

         rootNode.tick();

         update(rootNode);
      }
   }

   private void update(BehaviorTreeNodeExecutor<?, ?> node)
   {
      node.update();

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         update(child);
      }
   }

   public void destroy()
   {
      if (rootNode != null)
      {
         state.modifyTreeTopology(topologyOperationQueue -> topologyOperationQueue.queueDestroySubtree(rootNode));
      }
   }

   public void setRootNode(BehaviorTreeNodeLayer<?, ?, ?, ?> rootNode)
   {
      this.rootNode = (BehaviorTreeRootNodeExecutor) rootNode;
   }

   public BehaviorTreeRootNodeExecutor getRootNode()
   {
      return rootNode;
   }

   public BehaviorTreeState getState()
   {
      return state;
   }

   public CRDTInfo getCrdtInfo()
   {
      return crdtInfo;
   }

   public void loadBehavior(String jsonFileName)
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(saveFileDirectory, jsonFileName);
      if (file.getClasspathResource() != null)
      {
         state.modifyTreeTopology(topologyOperationQueue ->
         {
            BehaviorTreeNodeExecutor<?, ?> loadedNode = fileLoader.loadFromFile(file, topologyOperationQueue);

            if (loadedNode != null)
            {
               BehaviorTreeNodeExecutor<?, ?> nodeToInsert = loadedNode;

               if (state.getRootNode() == null) // Automatically add a root node if there isn't one
               {
                  nodeToInsert = new BehaviorTreeRootNodeExecutor(state.getAndIncrementNextID(),
                                                                  state.getCRDTInfo(),
                                                                  state.getSaveFileDirectory());
                  topologyOperationQueue.queueAddAndModifyNode(loadedNode, nodeToInsert);
               }

               var insertionDefinition = BehaviorTreeNodeInsertionDefinition.build(nodeToInsert,
                                                                                   state.getRootReferenceModification(),
                                                                                   this::setRootNode,
                                                                                   null,
                                                                                   BehaviorTreeNodeInsertionType.INSERT_ROOT);
               topologyOperationQueue.queueInsertNode(insertionDefinition);
            }
         });
      }
      else
      {
         LogTools.error("Cannot load behavior: {}", jsonFileName);
      }
   }

   public void deleteRootNode()
   {
      state.modifyTreeTopology(topologyOperationQueue -> topologyOperationQueue.queueDestroySubtree(rootNode));
      this.rootNode = null;
      state.getRootReferenceModification().modify();
   }
}
