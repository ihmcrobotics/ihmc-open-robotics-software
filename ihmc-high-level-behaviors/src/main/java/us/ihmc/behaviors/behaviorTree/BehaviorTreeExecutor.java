package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.condition.LLMConditionExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeExecutor extends BehaviorTree<BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>
{
   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2PeerClockOffsetEstimator peerClockEstimator,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      super(ROS2ActorDesignation.ROBOT,
            peerClockEstimator,
            new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees"),
            new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot));
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
            if (this.rootNode == null)
               topologyOperationQueue.queueDestroyEntireTree();

            BehaviorTreeRootNodeExecutor rootNode = (BehaviorTreeRootNodeExecutor) getNodeBuilder().createRootNode(getAndIncrementNextID());
            BehaviorTreeNodeExecutor<?, ?> loadedNode = getFileLoader().loadFromFile(rootNode, file, topologyOperationQueue);

            if (loadedNode != null)
            {
               rootNode.getDefinition().modify();
               topologyOperationQueue.queueSetRootNodeModify(rootNode);
               topologyOperationQueue.queueAppendChildModify(rootNode, loadedNode);
            }
         });
      }
      else
      {
         LogTools.error("Cannot load behavior: {}", jsonFileName);
      }
   }
}
