package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeBuilder<BehaviorTreeNodeExecutor<?, ?>>
{
   private final LogToolsLogger logToolsLogger = new LogToolsLogger();
   private CRDTInfo crdtInfo; // TODO: Make final somehow
   private WorkspaceResourceDirectory saveFileDirectory;
   private final DRCRobotModel robotModel;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
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

      // TODO: Probably create this in the BehaviorTree
      controllerStatusTracker = new ControllerStatusTracker(logToolsLogger, ros2ControllerHelper.getROS2Node(), robotModel.getSimpleRobotName());
   }

   @Override
   public void initialize(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      this.crdtInfo = crdtInfo;
      this.saveFileDirectory = saveFileDirectory;
   }

   @Override
   public BehaviorTreeRootNodeExecutor createRootNode(long id)
   {
      return new BehaviorTreeRootNodeExecutor(id,
                                              crdtInfo,
                                              saveFileDirectory,
                                              robotModel,
                                              ros2ControllerHelper,
                                              controllerStatusTracker,
                                              syncedRobot,
                                              referenceFrameLibrary,
                                              sceneGraph,
                                              detectionManager);
   }

   @Override
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>> rootNodeType)
   {
      BehaviorTreeRootNodeExecutor rootNode = (BehaviorTreeRootNodeExecutor) rootNodeType;

      if (nodeType == BehaviorTreeNodeDefinition.class) // TODO: Should not exist???
         return new BehaviorTreeNodeExecutor<>(id, rootNode);
      if (nodeType == AI2RNodeDefinition.class)
         return new AI2RNodeExecutor(id, rootNode);
      if (nodeType == ActionSequenceDefinition.class)
         return new ActionSequenceExecutor(id, rootNode);
      if (nodeType == FallbackNodeDefinition.class)
         return new FallbackNodeExecutor(id, rootNode);
      if (nodeType == ConditionNodeDefinition.class)
         return new ConditionNodeExecutor(id, rootNode);
      if (nodeType == GotoNodeDefinition.class)
         return new GotoNodeExecutor(id, rootNode);
      if (nodeType == CheckPointNodeDefinition.class)
         return new CheckPointNodeExecutor(id, rootNode);
      if (nodeType == DoorTraversalDefinition.class)
         return new DoorTraversalExecutor(id, rootNode);
      if (nodeType == BuildingExplorationDefinition.class)
         return new BuildingExplorationExecutor(id, rootNode);
      if (nodeType == ChestOrientationActionDefinition.class)
         return new ChestOrientationActionExecutor(id, rootNode);
      if (nodeType == FootstepPlanActionDefinition.class)
         return new FootstepPlanActionExecutor(id, rootNode);
      if (nodeType == HandPoseActionDefinition.class)
         return new HandPoseActionExecutor(id, rootNode);
      if (nodeType == HandWrenchActionDefinition.class)
         return new HandWrenchActionExecutor(id, rootNode);
      if (nodeType == ScrewPrimitiveActionDefinition.class)
         return new ScrewPrimitiveActionExecutor(id, rootNode);
      if (nodeType == PelvisHeightOrientationActionDefinition.class)
         return new PelvisHeightOrientationActionExecutor(id, rootNode);
      if (nodeType == SakeHandCommandActionDefinition.class)
         return new SakeHandCommandActionExecutor(id, rootNode);
      if (nodeType == WaitDurationActionDefinition.class)
         return new WaitDurationActionExecutor(id, rootNode);
      if (nodeType == FootPoseActionDefinition.class)
         return new FootPoseActionExecutor(id, rootNode);

      return null;
   }
}
