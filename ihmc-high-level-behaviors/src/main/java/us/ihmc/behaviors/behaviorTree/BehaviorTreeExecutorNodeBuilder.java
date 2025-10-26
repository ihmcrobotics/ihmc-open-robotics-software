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

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;

public class BehaviorTreeExecutorNodeBuilder implements BehaviorTreeNodeBuilder<BehaviorTreeNodeExecutor<?, ?>>
{
   private static final Map<Class<?>, BiFunction<Long, BehaviorTreeRootNodeExecutor, BehaviorTreeNodeExecutor<?, ?>>> MAP = new HashMap<>();
   static
   {
      MAP.put(BehaviorTreeNodeDefinition.class, BehaviorTreeNodeExecutor::new);
      MAP.put(AI2RNodeDefinition.class, AI2RNodeExecutor::new);
      MAP.put(ActionSequenceDefinition.class, ActionSequenceExecutor::new);
      MAP.put(FallbackNodeDefinition.class, FallbackNodeExecutor::new);
      MAP.put(ConditionNodeDefinition.class, ConditionNodeExecutor::new);
      MAP.put(GotoNodeDefinition.class, GotoNodeExecutor::new);
      MAP.put(CheckPointNodeDefinition.class, CheckPointNodeExecutor::new);
      MAP.put(DoorTraversalDefinition.class, DoorTraversalExecutor::new);
      MAP.put(BuildingExplorationDefinition.class, BuildingExplorationExecutor::new);
      MAP.put(ChestOrientationActionDefinition.class, ChestOrientationActionExecutor::new);
      MAP.put(FootstepPlanActionDefinition.class, FootstepPlanActionExecutor::new);
      MAP.put(HandPoseActionDefinition.class, HandPoseActionExecutor::new);
      MAP.put(HandWrenchActionDefinition.class, HandWrenchActionExecutor::new);
      MAP.put(ScrewPrimitiveActionDefinition.class, ScrewPrimitiveActionExecutor::new);
      MAP.put(PelvisHeightOrientationActionDefinition.class, PelvisHeightOrientationActionExecutor::new);
      MAP.put(SakeHandCommandActionDefinition.class, SakeHandCommandActionExecutor::new);
      MAP.put(WaitDurationActionDefinition.class, WaitDurationActionExecutor::new);
      MAP.put(FootPoseActionDefinition.class, FootPoseActionExecutor::new);
   }

   private final LogToolsLogger logToolsLogger = new LogToolsLogger();
   private CRDTInfo crdtInfo; // TODO: Make final somehow
   private WorkspaceResourceDirectory saveFileDirectory;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ControllerStatusTracker controllerStatusTracker;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
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
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>> rootNode)
   {
      if (MAP.containsKey(nodeType))
         return MAP.get(nodeType).apply(id, (BehaviorTreeRootNodeExecutor) rootNode);

      return null;
   }
}
