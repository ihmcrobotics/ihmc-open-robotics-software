package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.ai2r.AI2RNodeExecutor;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionExecutor;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.door.DoorTraversalExecutor;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceExecutor;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeStateBuilder
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final LogToolsLogger logToolsLogger = new LogToolsLogger();
   private final ControllerStatusTracker controllerStatusTracker;
   private final WalkingFootstepTracker footstepTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final SceneGraph sceneGraph;
   private final DetectionManager detectionManager;

   public BehaviorTreeExecutorNodeBuilder(DRCRobotModel robotModel,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ROS2SyncedRobotModel syncedRobot,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          SceneGraph sceneGraph,
                                          DetectionManager detectionManager)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.sceneGraph = sceneGraph;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.detectionManager = detectionManager;

      controllerStatusTracker = new ControllerStatusTracker(logToolsLogger, ros2ControllerHelper.getROS2NodeInterface(), robotModel.getSimpleRobotName());
      footstepTracker = controllerStatusTracker.getFootstepTracker();
      walkingControllerParameters = robotModel.getWalkingControllerParameters();
   }

   @Override
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      if (nodeType == BehaviorTreeRootNodeDefinition.class)
      {
         return new BehaviorTreeRootNodeExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == BehaviorTreeNodeDefinition.class)
      {
         return new BehaviorTreeNodeExecutor<>(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == AI2RNodeDefinition.class)
      {
         return new AI2RNodeExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, sceneGraph);
      }
      if (nodeType == ActionSequenceDefinition.class)
      {
         return new ActionSequenceExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == DoorTraversalDefinition.class)
      {
         return new DoorTraversalExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, sceneGraph);
      }
      if (nodeType == TrashCanInteractionDefinition.class)
      {
         return new TrashCanInteractionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, sceneGraph);
      }
      if (nodeType == BuildingExplorationDefinition.class)
      {
         return new BuildingExplorationExecutor(id, crdtInfo, saveFileDirectory, sceneGraph);
      }
      if (nodeType == ChestOrientationActionDefinition.class)
      {
         return new ChestOrientationActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, referenceFrameLibrary);
      }
      if (nodeType == FootstepPlanActionDefinition.class)
      {
         return new FootstepPlanActionExecutor(id,
                                               crdtInfo,
                                               saveFileDirectory,
                                               ros2ControllerHelper,
                                               syncedRobot,
                                               controllerStatusTracker,
                                               referenceFrameLibrary,
                                               walkingControllerParameters);
      }
      if (nodeType == HandPoseActionDefinition.class)
      {
         return new HandPoseActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, robotModel, syncedRobot);
      }
      if (nodeType == HandWrenchActionDefinition.class)
      {
         return new HandWrenchActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper);
      }
      if (nodeType == ScrewPrimitiveActionDefinition.class)
      {
         return new ScrewPrimitiveActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, robotModel, syncedRobot);
      }
      if (nodeType == PelvisHeightOrientationActionDefinition.class)
      {
         return new PelvisHeightOrientationActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, syncedRobot);
      }
      if (nodeType == SakeHandCommandActionDefinition.class)
      {
         return new SakeHandCommandActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot);
      }
      if (nodeType == WaitDurationActionDefinition.class)
      {
         return new WaitDurationActionExecutor(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == FootPoseActionDefinition.class)
      {
         return new FootPoseActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, syncedRobot);
      }

      return null;
   }
}
