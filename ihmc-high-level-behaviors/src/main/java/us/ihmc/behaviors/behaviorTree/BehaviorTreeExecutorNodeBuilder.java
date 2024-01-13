package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceExecutor;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeStateBuilder
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final SceneGraph sceneGraph;
   private final WalkingFootstepTracker footstepTracker;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ROS2ControllerHelper ros2ControllerHelper;

   public BehaviorTreeExecutorNodeBuilder(DRCRobotModel robotModel,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ROS2SyncedRobotModel syncedRobot,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          SceneGraph sceneGraph)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.sceneGraph = sceneGraph;
      this.ros2ControllerHelper = ros2ControllerHelper;

      footstepTracker = new WalkingFootstepTracker(ros2ControllerHelper.getROS2NodeInterface(), robotModel.getSimpleRobotName());
      footstepPlanner = new FootstepPlanningModule(FootstepPlanningModule.class.getSimpleName());
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      walkingControllerParameters = robotModel.getWalkingControllerParameters();
   }

   @Override
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      if (nodeType == BehaviorTreeNodeDefinition.class)
      {
         return new BehaviorTreeNodeExecutor<>(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == ActionSequenceDefinition.class)
      {
         return new ActionSequenceExecutor(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == ArmJointAnglesActionDefinition.class)
      {
         return new ArmJointAnglesActionExecutor(id, crdtInfo, saveFileDirectory, robotModel, ros2ControllerHelper, syncedRobot);
      }
      if (nodeType == ChestOrientationActionDefinition.class)
      {
         return new ChestOrientationActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, syncedRobot, referenceFrameLibrary, sceneGraph);
      }
      if (nodeType == FootstepPlanActionDefinition.class)
      {
         return new FootstepPlanActionExecutor(id,
                                               crdtInfo,
                                               saveFileDirectory,
                                               ros2ControllerHelper,
                                               syncedRobot,
                                               footstepTracker,
                                               referenceFrameLibrary,
                                               sceneGraph,
                                               walkingControllerParameters);
      }
      if (nodeType == HandPoseActionDefinition.class)
      {
         return new HandPoseActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, sceneGraph, robotModel, syncedRobot);
      }
      if (nodeType == HandWrenchActionDefinition.class)
      {
         return new HandWrenchActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper);
      }
      if (nodeType == PelvisHeightPitchActionDefinition.class)
      {
         return new PelvisHeightPitchActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper, referenceFrameLibrary, sceneGraph, syncedRobot);
      }
      if (nodeType == SakeHandCommandActionDefinition.class)
      {
         return new SakeHandCommandActionExecutor(id, crdtInfo, saveFileDirectory, ros2ControllerHelper);
      }
      if (nodeType == WaitDurationActionDefinition.class)
      {
         return new WaitDurationActionExecutor(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == WalkActionDefinition.class)
      {
         return new WalkActionExecutor(id,
                                       crdtInfo,
                                       saveFileDirectory,
                                       ros2ControllerHelper,
                                       syncedRobot,
                                       footstepTracker,
                                       footstepPlanner,
                                       footstepPlannerParameters,
                                       walkingControllerParameters,
                                       referenceFrameLibrary,
                                       sceneGraph);
      }

      return null;
   }
}
