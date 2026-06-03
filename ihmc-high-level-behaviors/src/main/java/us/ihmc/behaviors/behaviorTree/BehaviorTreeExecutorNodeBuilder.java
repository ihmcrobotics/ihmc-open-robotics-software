package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.jros2.ROS2Node;

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
      REGISTRY.put(CheckpointNodeDefinition.class, CheckpointNodeExecutor::new);
      REGISTRY.put(SceneActionDefinition.class, SceneActionExecutor::new);
      REGISTRY.put(MimicActionDefinition.class, MimicActionExecutor::new);
      REGISTRY.put(AI2RNodeDefinition.class, AI2RNodeExecutor::new);
      REGISTRY.put(DoorTraversalDefinition.class, DoorTraversalExecutor::new);
      REGISTRY.put(BuildingExplorationDefinition.class, BuildingExplorationExecutor::new);
      REGISTRY.put(NeckActionDefinition.class, NeckActionExecutor::new);
      REGISTRY.put(SpineActionDefinition.class, SpineActionExecutor::new);
      REGISTRY.put(WalkActionDefinition.class, WalkActionExecutor::new);
      REGISTRY.put(ArmActionDefinition.class, ArmActionExecutor::new);
      REGISTRY.put(HandWrenchActionDefinition.class, HandWrenchActionExecutor::new);
      REGISTRY.put(PelvisActionDefinition.class, PelvisActionExecutor::new);
      REGISTRY.put(AbilityHandActionDefinition.class, AbilityHandActionExecutor::new);
      REGISTRY.put(EZGripperActionDefinition.class, EZGripperActionExecutor::new);
      REGISTRY.put(WaitActionDefinition.class, WaitActionExecutor::new);
      REGISTRY.put(LegActionDefinition.class, LegActionExecutor::new);
   }

   private BehaviorTreeExecutor tree;
   private WorkspaceResourceDirectory saveFileDirectory;
   private ROS2ControllerHelper ros2ControllerHelper;
   private TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder;
   private ROS2SyncedRobotModel syncedRobot;
   private ControllerStatusTracker controllerStatusTracker;
   private SideDependentList<AbilityHandActionComms> abilityHandComms;
   private ImageSensor imageSensor;
   private YOLOv8DetectionExecutor yolo;
   private IsaacROSFoundationPoseCommunicatorMap foundationPose;
   private TerrainMapData terrainMapData;

   public void initialize(BehaviorTreeExecutor tree,
                          WorkspaceResourceDirectory saveFileDirectory,
                          ROS2ControllerHelper ros2ControllerHelper,
                          TriFunction<DRCRobotModel, ROS2Node, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
                          ROS2SyncedRobotModel syncedRobot,
                          ControllerStatusTracker controllerStatusTracker,
                          SideDependentList<AbilityHandActionComms> abilityHandComms,
                          ImageSensor imageSensor,
                          YOLOv8DetectionExecutor yolo,
                          IsaacROSFoundationPoseCommunicatorMap foundationPose,
                          TerrainMapData terrainMapData)
   {
      this.tree = tree;
      this.saveFileDirectory = saveFileDirectory;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.kinematicsSimulationBuilder = kinematicsSimulationBuilder;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.abilityHandComms = abilityHandComms;
      this.imageSensor = imageSensor;
      this.yolo = yolo;
      this.foundationPose = foundationPose;
      this.terrainMapData = terrainMapData;
   }

   @Override
   public BehaviorTreeRootNodeExecutor createRootNode(long id)
   {
      BehaviorTreeSceneExecutor scene = new BehaviorTreeSceneExecutor(tree.getCRDTInfo(),
                                                                      tree::getAndIncrementNextID,
                                                                      syncedRobot,
                                                                      imageSensor,
                                                                      yolo,
                                                                      foundationPose,
                                                                      terrainMapData);
      return new BehaviorTreeRootNodeExecutor(id,
                                              tree,
                                              saveFileDirectory,
                                              ros2ControllerHelper,
                                              kinematicsSimulationBuilder,
                                              syncedRobot,
                                              controllerStatusTracker,
                                              abilityHandComms,
                                              scene);
   }

   @Override
   public BehaviorTreeNodeExecutor<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>> rootNode)
   {
      if (REGISTRY.containsKey(nodeType))
         return REGISTRY.get(nodeType).apply(id, (BehaviorTreeRootNodeExecutor) rootNode);

      return null;
   }
}
