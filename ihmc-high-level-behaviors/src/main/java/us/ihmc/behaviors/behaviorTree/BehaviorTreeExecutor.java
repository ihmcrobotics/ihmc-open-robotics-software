package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeExecutor
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, (int) ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final BehaviorTreeExecutorNodeBuilder nodeBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final BehaviorTreeState state;
   private BehaviorTreeRootNodeExecutor rootNode;
   private final BehaviorTreeFileLoader<BehaviorTreeNodeExecutor<?, ?>> fileLoader;
   private final WorkspaceResourceDirectory saveFileDirectory = new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees");

   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               SceneGraph sceneGraph,
                               DetectionManager detectionManager,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      nodeBuilder = new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot, referenceFrameLibrary, sceneGraph, detectionManager);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode, crdtInfo);

      state = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode, crdtInfo, null);
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
                  topologyOperationQueue.queueAddAndFreezeNode(loadedNode, nodeToInsert);
               }

               var insertionDefinition = BehaviorTreeNodeInsertionDefinition.build(nodeToInsert,
                                                                                   state,
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
}
