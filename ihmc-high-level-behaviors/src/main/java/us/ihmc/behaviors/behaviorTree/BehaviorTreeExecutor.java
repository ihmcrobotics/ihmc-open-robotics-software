package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.condition.LLMConditionExecutor;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeExecutor extends BehaviorTree<BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>
{
   private final ControllerStatusTracker controllerStatusTracker;
   private final BehaviorTreeSceneExecutor scene;
   private final SideDependentList<AbilityHandActionComms> abilityHandComms = new SideDependentList<>();

   public BehaviorTreeExecutor(ROS2SyncedRobotModel syncedRobot,
                               ROS2PeerClockOffsetEstimator peerClockEstimator,
                               ROS2ControllerHelper ros2ControllerHelper,
                               YOLOv8DetectionExecutor yolo,
                               IsaacROSFoundationPoseCommunicatorMap foundationPose)
   {
      super(syncedRobot,
            ROS2ActorDesignation.ROBOT,
            peerClockEstimator,
            new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees"),
            new BehaviorTreeExecutorNodeBuilder());

      controllerStatusTracker = new ControllerStatusTracker(new LogToolsLogger(), ros2ControllerHelper.getROS2Node(), robotModel.getSimpleRobotName());
      for (RobotSide robotSide : RobotSide.values)
         abilityHandComms.put(robotSide, new AbilityHandActionComms(robotSide, ros2ControllerHelper.getROS2Node()));
      scene = new BehaviorTreeSceneExecutor(crdtInfo, this::getAndIncrementNextID, syncedRobot, yolo, foundationPose);
      setScene(scene);

      ((BehaviorTreeExecutorNodeBuilder) getNodeBuilder()).initialize(crdtInfo,
                                                                      saveFileDirectory,
                                                                      ros2ControllerHelper,
                                                                      syncedRobot,
                                                                      controllerStatusTracker,
                                                                      abilityHandComms,
                                                                      scene);
   }

   public void update()
   {
      scene.update();

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
