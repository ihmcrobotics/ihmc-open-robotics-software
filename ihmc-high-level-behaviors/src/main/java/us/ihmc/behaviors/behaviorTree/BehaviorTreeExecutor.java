package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.condition.LLMConditionExecutor;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.nio.file.Files;
import java.nio.file.Path;

public class BehaviorTreeExecutor extends BehaviorTree<BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>
{
   private final ControllerStatusTracker controllerStatusTracker;
   private final SideDependentList<AbilityHandActionComms> abilityHandComms = new SideDependentList<>();

   public BehaviorTreeExecutor(
         ROS2SyncedRobotModel syncedRobot,
         ROS2PeerClockOffsetEstimator peerClockEstimator,
         ROS2ControllerHelper ros2ControllerHelper,
         TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ImageSensor imageSensor,
         YOLOv8DetectionExecutor yolo,
         IsaacROSFoundationPoseCommunicatorMap foundationPose,
         TerrainMapData terrainMapData)
   {
      super(syncedRobot,
            ROS2ActorDesignation.ROBOT,
            peerClockEstimator,
            new WorkspaceResourceDirectory(BehaviorTreeExecutor.class, "/behaviorTrees"),
            new BehaviorTreeExecutorNodeBuilder());

      controllerStatusTracker = new ControllerStatusTracker(new LogToolsLogger(), ros2ControllerHelper.getROS2Node(), syncedRobot);
      for (RobotSide robotSide : RobotSide.values)
         abilityHandComms.put(robotSide, new AbilityHandActionComms(robotSide, ros2ControllerHelper.getROS2Node()));

      ((BehaviorTreeExecutorNodeBuilder) getNodeBuilder()).initialize(this,
                                                                      saveFileDirectory,
                                                                      ros2ControllerHelper,
                                                                      kinematicsSimulationBuilder,
                                                                      syncedRobot,
                                                                      controllerStatusTracker,
                                                                      abilityHandComms,
                                                                      imageSensor,
                                                                      yolo,
                                                                      foundationPose,
                                                                      terrainMapData);
   }

   public void update()
   {
      for (RobotSide side : abilityHandComms.sides())
         abilityHandComms.get(side).update();

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

   public void setTreeDirectory(Path directory)
   {
      if (directory != null)
         getSaveFileDirectory().setFilesystemDirectory(directory);
   }

   public boolean canLoadBehavior(String jsonFileName)
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(getSaveFileDirectory(), jsonFileName);
      return (file.isFileAccessAvailable() && Files.exists(file.getFilesystemFile())) || file.getClasspathResource() != null;
   }

   /** @return false when the file is missing or unreadable. The current tree is left in place. */
   public boolean loadBehavior(String jsonFileName)
   {
      if (!canLoadBehavior(jsonFileName))
      {
         LogTools.error("Cannot load behavior: {}", jsonFileName);
         return false;
      }

      boolean[] loaded = {false};
      WorkspaceResourceFile file = new WorkspaceResourceFile(getSaveFileDirectory(), jsonFileName);
      modifyTreeTopology(topologyOperationQueue ->
      {
         if (rootNode != null)
            topologyOperationQueue.queueDestroyEntireTreeModify();

         BehaviorTreeRootNodeExecutor rootNode = (BehaviorTreeRootNodeExecutor) getNodeBuilder().createRootNode(getAndIncrementNextID());
         BehaviorTreeNodeExecutor<?, ?> loadedNode = getFileLoader().loadFromFile(rootNode, file, topologyOperationQueue);
         if (loadedNode != null)
         {
            rootNode.getState().setAutomaticExecution(true);
            rootNode.getDefinition().modify();
            topologyOperationQueue.queueSetRootNodeModify(rootNode);
            topologyOperationQueue.queueAppendChildModify(rootNode, loadedNode);
            loaded[0] = true;
         }
      });
      return loaded[0];
   }

}
