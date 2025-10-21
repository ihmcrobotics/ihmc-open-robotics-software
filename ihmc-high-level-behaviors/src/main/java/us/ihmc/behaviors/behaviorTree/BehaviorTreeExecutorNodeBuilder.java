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
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeBuilder<BehaviorTreeNodeExecutor<?, ?>>
{
   private static final Map<Class<?>, BiFunction<Long, BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>> REGISTRY = new HashMap<>();
   static
   {
      REGISTRY.put(BehaviorTreeNodeDefinition.class, BehaviorTreeNodeExecutor::new);
      REGISTRY.put(AI2RNodeDefinition.class, AI2RNodeExecutor::new);
      REGISTRY.put(ActionSequenceDefinition.class, ActionSequenceExecutor::new);
      REGISTRY.put(FallbackNodeDefinition.class, FallbackNodeExecutor::new);
      REGISTRY.put(ConditionNodeDefinition.class, ConditionNodeExecutor::new);
      REGISTRY.put(GotoNodeDefinition.class, GotoNodeExecutor::new);
      REGISTRY.put(CheckPointNodeDefinition.class, CheckPointNodeExecutor::new);
      REGISTRY.put(DoorTraversalDefinition.class, DoorTraversalExecutor::new);
      REGISTRY.put(BuildingExplorationDefinition.class, BuildingExplorationExecutor::new);
      REGISTRY.put(ChestOrientationActionDefinition.class, ChestOrientationActionExecutor::new);
      REGISTRY.put(FootstepPlanActionDefinition.class, FootstepPlanActionExecutor::new);
      REGISTRY.put(HandPoseActionDefinition.class, HandPoseActionExecutor::new);
      REGISTRY.put(HandWrenchActionDefinition.class, HandWrenchActionExecutor::new);
      REGISTRY.put(ScrewPrimitiveActionDefinition.class, ScrewPrimitiveActionExecutor::new);
      REGISTRY.put(PelvisHeightOrientationActionDefinition.class, PelvisHeightOrientationActionExecutor::new);
      REGISTRY.put(SakeHandCommandActionDefinition.class, SakeHandCommandActionExecutor::new);
      REGISTRY.put(WaitDurationActionDefinition.class, WaitDurationActionExecutor::new);
      REGISTRY.put(FootPoseActionDefinition.class, FootPoseActionExecutor::new);
   }

   private final LogToolsLogger logToolsLogger = new LogToolsLogger();
   private CRDTInfo crdtInfo; // TODO: Make final somehow
   private WorkspaceResourceDirectory saveFileDirectory;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ControllerStatusTracker controllerStatusTracker;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final DetectionManager detectionManager;

   public BehaviorTreeExecutorNodeBuilder(DRCRobotModel robotModel,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ROS2SyncedRobotModel syncedRobot,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          DetectionManager detectionManager)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
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
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>> rootNode)
   {
      if (REGISTRY.containsKey(nodeType))
         return REGISTRY.get(nodeType).apply(id, (BehaviorTreeRootNodeExecutor) rootNode);

      return null;
   }
}
