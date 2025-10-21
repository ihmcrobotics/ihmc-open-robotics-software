package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeExecutor;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationExecutor;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeExecutor;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeExecutor;
import us.ihmc.behaviors.logic.GotoNodeDefinition;
import us.ihmc.behaviors.logic.GotoNodeExecutor;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceExecutor;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ChestOrientationActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.FootPoseActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FootPoseActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.HandPoseActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.HandWrenchActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.PelvisHeightOrientationActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.PelvisHeightOrientationActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SakeHandCommandActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ScrewPrimitiveActionExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionExecutor;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeBuilder<BehaviorTreeNodeExecutor<?, ?>>
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

      controllerStatusTracker = new ControllerStatusTracker(logToolsLogger, ros2ControllerHelper.getROS2Node(), robotModel.getSimpleRobotName());
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
      if (nodeType == FallbackNodeDefinition.class)
      {
         return new FallbackNodeExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == ConditionNodeDefinition.class)
      {
         return new ConditionNodeExecutor(id, crdtInfo, saveFileDirectory, referenceFrameLibrary);
      }
      if (nodeType == GotoNodeDefinition.class)
      {
         return new GotoNodeExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == CheckPointNodeDefinition.class)
      {
         return new CheckPointNodeExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == DoorTraversalDefinition.class)
      {
         return new DoorTraversalExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, sceneGraph);
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
