package us.ihmc.rdx.behaviorTree;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.*;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.rdx.behaviorTree.actions.*;
import us.ihmc.rdx.behaviorTree.condition.*;
import us.ihmc.rdx.behaviorTree.control.*;
import us.ihmc.rdx.behaviorTree.scene.RDXBehaviorTreeScene;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;

public class RDXBehaviorTreeNodeBuilder implements BehaviorTreeNodeBuilder<RDXBehaviorTreeNode<?, ?>>
{
   private static final Map<Class<?>, BiFunction<Long, RDXBehaviorTreeRootNode, RDXBehaviorTreeNode<?, ?>>> REGISTRY = new HashMap<>();
   static
   {
      REGISTRY.put(BehaviorTreeNodeDefinition.class, RDXBehaviorTreeNode::new);
      REGISTRY.put(ActionSequenceDefinition.class, RDXActionSequence::new);
      REGISTRY.put(FallbackNodeDefinition.class, RDXFallbackNode::new);
      REGISTRY.put(ConditionNodeDefinition.class, RDXConditionNode::new);
      REGISTRY.put(GotoNodeDefinition.class, RDXGotoNode::new);
      REGISTRY.put(CheckpointNodeDefinition.class, RDXCheckpointNode::new);
      REGISTRY.put(SceneActionDefinition.class, RDXSceneAction::new);
      REGISTRY.put(MimicActionDefinition.class, RDXMimicAction::new);
      REGISTRY.put(AI2RNodeDefinition.class, RDXAI2RNode::new);
      REGISTRY.put(DoorTraversalDefinition.class, RDXDoorTraversal::new);
      REGISTRY.put(BuildingExplorationDefinition.class, RDXBuildingExploration::new);
      REGISTRY.put(NeckActionDefinition.class, RDXNeckAction::new);
      REGISTRY.put(SpineActionDefinition.class, RDXSpineAction::new);
      REGISTRY.put(WalkActionDefinition.class, RDXWalkAction::new);
      REGISTRY.put(ArmActionDefinition.class, RDXArmAction::new);
      REGISTRY.put(HandWrenchActionDefinition.class, RDXHandWrenchAction::new);
      REGISTRY.put(ScrewPrimitiveActionDefinition.class, RDXScrewPrimitiveAction::new);
      REGISTRY.put(PelvisActionDefinition.class, RDXPelvisAction::new);
      REGISTRY.put(AbilityHandActionDefinition.class, RDXAbilityHandAction::new);
      REGISTRY.put(EZGripperActionDefinition.class, RDXEZGripperAction::new);
      REGISTRY.put(WaitActionDefinition.class, RDXWaitAction::new);
      REGISTRY.put(LegActionDefinition.class, RDXLegAction::new);
   }

   private RDXBehaviorTree tree;
   private WorkspaceResourceDirectory saveFileDirectory;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotCollisionModel selectionCollisionModel;
   private RDXBaseUI baseUI;
   private RDX3DPanel panel3D;

   public void initialize(RDXBehaviorTree tree,
                          WorkspaceResourceDirectory saveFileDirectory,
                          ROS2SyncedRobotModel syncedRobot,
                          RobotCollisionModel selectionCollisionModel,
                          RDXBaseUI baseUI,
                          RDX3DPanel panel3D)
   {
      this.tree = tree;
      this.saveFileDirectory = saveFileDirectory;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
   }

   @Override
   public BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>> createRootNode(long id)
   {
      RDXBehaviorTreeScene scene = new RDXBehaviorTreeScene(tree.getCRDTInfo(),
                                                            tree::getAndIncrementNextID,
                                                            syncedRobot,
                                                            baseUI);
      return new RDXBehaviorTreeRootNode(id,
                                         tree,
                                         saveFileDirectory,
                                         syncedRobot,
                                         scene,
                                         selectionCollisionModel,
                                         baseUI,
                                         panel3D);
   }

   @Override
   public RDXBehaviorTreeNode<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>> rootNodeType)
   {
      if (REGISTRY.containsKey(nodeType))
         return REGISTRY.get(nodeType).apply(id, (RDXBehaviorTreeRootNode) rootNodeType);

      return null;
   }

   // This method is in this class because we have a syncedRobot here.
   public void initializeActionNode(@Nullable RDXBehaviorTreeRootNode actionSequence,
                                    RDXActionNode<?, ?> newAction,
                                    int insertionIndex,
                                    RobotSide sideOfNewAction)
   {
      ActionNodeInitialization.initializeAction(actionSequence == null ? null : actionSequence.getState(),
                                                newAction.getState(),
                                                insertionIndex,
                                                sideOfNewAction,
                                                syncedRobot);
   }
}
