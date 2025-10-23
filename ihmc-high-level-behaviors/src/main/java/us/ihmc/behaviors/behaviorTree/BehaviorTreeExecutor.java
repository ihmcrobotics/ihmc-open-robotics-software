package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.condition.LLMConditionExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeExecutor extends BehaviorTree<BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>
{
   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2PeerClockOffsetEstimator peerClockEstimator,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               SceneGraph sceneGraph,
                               DetectionManager detectionManager,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      super(ROS2ActorDesignation.ROBOT,
            peerClockEstimator,
            new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees"),
            new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot, referenceFrameLibrary, sceneGraph, detectionManager));
   }

   @Override
   public BehaviorTreeRootNodeExecutor createRootNode(long id)
   {
      return new BehaviorTreeRootNodeExecutor(id, crdtInfo, saveFileDirectory);
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
      modifyTreeTopology(BehaviorTreeTopologyOperationQueue::queueDestroyEntireTree);
      LLMConditionExecutor.destroy();
   }

   public void loadBehavior(String jsonFileName)
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(getSaveFileDirectory(), jsonFileName);
      if (file.getClasspathResource() != null)
      {
         modifyTreeTopology(topologyOperationQueue ->
         {
            BehaviorTreeNodeExecutor<?, ?> loadedNode = getFileLoader().loadFromFile(file, topologyOperationQueue);

            if (loadedNode != null)
            {
               if (loadedNode instanceof BehaviorTreeRootNodeExecutor loadedRootNode) // If we loaded a root node, replace the existing one
               {
                  topologyOperationQueue.queueSetRootNodeModify(loadedRootNode);
               }
               else if (rootNode == null) // Automatically add a root node if there isn't one
               {
                  BehaviorTreeRootNodeExecutor newRootNode = new BehaviorTreeRootNodeExecutor(getAndIncrementNextID(),
                                                                                              getCRDTInfo(),
                                                                                              getSaveFileDirectory());
                  newRootNode.getDefinition().modify();
                  topologyOperationQueue.queueAppendChildModify(newRootNode, loadedNode);
                  topologyOperationQueue.queueSetRootNodeModify(newRootNode);
               }
               else // Add the loaded node as a child of the root node
               {
                  topologyOperationQueue.queueAppendChildModify((BehaviorTreeNodeExecutor<?, ?>) rootNode, loadedNode);
               }
            }
         });
      }
      else
      {
         LogTools.error("Cannot load behavior: {}", jsonFileName);
      }
   }
}
