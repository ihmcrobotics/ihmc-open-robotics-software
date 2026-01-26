package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.yolo.YOLOTerrainMapIntegrator;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.SideDependentList;
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
      REGISTRY.put(ActionSequenceDefinition.class, ActionSequenceExecutor::new);
      REGISTRY.put(FallbackNodeDefinition.class, FallbackNodeExecutor::new);
      REGISTRY.put(ConditionNodeDefinition.class, ConditionNodeExecutor::new);
      REGISTRY.put(GotoNodeDefinition.class, GotoNodeExecutor::new);
      REGISTRY.put(CheckPointNodeDefinition.class, CheckPointNodeExecutor::new);
      REGISTRY.put(SceneActionNodeDefinition.class, SceneActionNodeExecutor::new);
      REGISTRY.put(AI2RNodeDefinition.class, AI2RNodeExecutor::new);
      REGISTRY.put(DoorTraversalDefinition.class, DoorTraversalExecutor::new);
      REGISTRY.put(BuildingExplorationDefinition.class, BuildingExplorationExecutor::new);
      REGISTRY.put(NeckActionDefinition.class, NeckActionExecutor::new);
      REGISTRY.put(ChestOrientationActionDefinition.class, ChestOrientationActionExecutor::new);
      REGISTRY.put(FootstepPlanActionDefinition.class, FootstepPlanActionExecutor::new);
      REGISTRY.put(HandPoseActionDefinition.class, HandPoseActionExecutor::new);
      REGISTRY.put(HandWrenchActionDefinition.class, HandWrenchActionExecutor::new);
      REGISTRY.put(ScrewPrimitiveActionDefinition.class, ScrewPrimitiveActionExecutor::new);
      REGISTRY.put(PelvisHeightOrientationActionDefinition.class, PelvisHeightOrientationActionExecutor::new);
      REGISTRY.put(AbilityHandActionDefinition.class, AbilityHandActionExecutor::new);
      REGISTRY.put(SakeHandCommandActionDefinition.class, SakeHandCommandActionExecutor::new);
      REGISTRY.put(WaitDurationActionDefinition.class, WaitDurationActionExecutor::new);
      REGISTRY.put(FootPoseActionDefinition.class, FootPoseActionExecutor::new);
   }

   private CRDTInfo crdtInfo;
   private WorkspaceResourceDirectory saveFileDirectory;
   private ROS2ControllerHelper ros2ControllerHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private ControllerStatusTracker controllerStatusTracker;
   private SideDependentList<AbilityHandActionComms> abilityHandComms;
   private BehaviorTreeSceneExecutor scene;
   private TerrainMapData terrainMapData;

   public void initialize(CRDTInfo crdtInfo,
                          WorkspaceResourceDirectory saveFileDirectory,
                          ROS2ControllerHelper ros2ControllerHelper,
                          ROS2SyncedRobotModel syncedRobot,
                          ControllerStatusTracker controllerStatusTracker,
                          SideDependentList<AbilityHandActionComms> abilityHandComms,
                          BehaviorTreeSceneExecutor scene,
                          TerrainMapData terrainMapData)
   {
      this.crdtInfo = crdtInfo;
      this.saveFileDirectory = saveFileDirectory;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.abilityHandComms = abilityHandComms;
      this.scene = scene;
      this.terrainMapData = terrainMapData;
   }

   @Override
   public BehaviorTreeRootNodeExecutor createRootNode(long id)
   {
      return new BehaviorTreeRootNodeExecutor(id,
                                              crdtInfo,
                                              saveFileDirectory,
                                              ros2ControllerHelper,
                                              syncedRobot,
                                              controllerStatusTracker,
                                              abilityHandComms,
                                              scene,
                                              terrainMapData);
   }

   @Override
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>> rootNode)
   {
      if (REGISTRY.containsKey(nodeType))
         return REGISTRY.get(nodeType).apply(id, (BehaviorTreeRootNodeExecutor) rootNode);

      return null;
   }
}
