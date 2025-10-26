package us.ihmc.rdx.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.*;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.behaviorTree.actions.*;
import us.ihmc.rdx.behaviorTree.condition.*;
import us.ihmc.rdx.behaviorTree.control.*;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;

public class RDXBehaviorTreeNodeBuilder implements BehaviorTreeNodeBuilder<RDXBehaviorTreeNode<?, ?>>
{
   private static final Map<Class<?>, BiFunction<Long, RDXBehaviorTreeRootNode, RDXBehaviorTreeNode<?, ?>>> MAP = new HashMap<>();
   static
   {
      MAP.put(BehaviorTreeNodeDefinition.class, RDXBehaviorTreeNode::new);
      MAP.put(AI2RNodeDefinition.class, RDXAI2RNode::new);
      MAP.put(ActionSequenceDefinition.class, RDXActionSequence::new);
      MAP.put(FallbackNodeDefinition.class, RDXFallbackNode::new);
      MAP.put(ConditionNodeDefinition.class, RDXConditionNode::new);
      MAP.put(GotoNodeDefinition.class, RDXGotoNode::new);
      MAP.put(CheckPointNodeDefinition.class, RDXCheckPointNode::new);
      MAP.put(DoorTraversalDefinition.class, RDXDoorTraversal::new);
      MAP.put(BuildingExplorationDefinition.class, RDXBuildingExploration::new);
      MAP.put(ChestOrientationActionDefinition.class, RDXChestOrientationAction::new);
      MAP.put(FootstepPlanActionDefinition.class, RDXFootstepPlanAction::new);
      MAP.put(HandPoseActionDefinition.class, RDXHandPoseAction::new);
      MAP.put(HandWrenchActionDefinition.class, RDXHandWrenchAction::new);
      MAP.put(ScrewPrimitiveActionDefinition.class, RDXScrewPrimitiveAction::new);
      MAP.put(PelvisHeightOrientationActionDefinition.class, RDXPelvisHeightOrientationAction::new);
      MAP.put(SakeHandCommandActionDefinition.class, RDXSakeHandCommandAction::new);
      MAP.put(WaitDurationActionDefinition.class, RDXWaitDurationAction::new);
      MAP.put(FootPoseActionDefinition.class, RDXFootPoseAction::new);
   }

   private CRDTInfo crdtInfo; // TODO: Make final somehow
   private WorkspaceResourceDirectory saveFileDirectory;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final RobotCollisionModel selectionCollisionModel;
   private final RDXBaseUI baseUI;
   private final RDX3DPanel panel3D;

   public RDXBehaviorTreeNodeBuilder(DRCRobotModel robotModel,
                                     ROS2SyncedRobotModel syncedRobot,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     RobotCollisionModel selectionCollisionModel,
                                     RDXBaseUI baseUI,
                                     RDX3DPanel panel3D)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
   }

   @Override
   public void initialize(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      this.crdtInfo = crdtInfo;
      this.saveFileDirectory = saveFileDirectory;
   }

   @Override
   public BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>> createRootNode(long id)
   {
      return new RDXBehaviorTreeRootNode(id,
                                         crdtInfo,
                                         saveFileDirectory,
                                         robotModel,
                                         referenceFrameLibrary,
                                         syncedRobot,
                                         selectionCollisionModel,
                                         baseUI,
                                         panel3D);
   }

   @Override
   public RDXBehaviorTreeNode<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>> rootNodeType)
   {
      if (MAP.containsKey(nodeType))
         return MAP.get(nodeType).apply(id, (RDXBehaviorTreeRootNode) rootNodeType);

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
