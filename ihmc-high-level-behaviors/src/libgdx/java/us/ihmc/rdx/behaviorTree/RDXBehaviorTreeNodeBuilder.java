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

public class RDXBehaviorTreeNodeBuilder implements BehaviorTreeNodeBuilder<RDXBehaviorTreeNode<?, ?>>
{
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
      RDXBehaviorTreeRootNode rootNode = (RDXBehaviorTreeRootNode) rootNodeType;

      if (nodeType == BehaviorTreeNodeDefinition.class) // TODO: Should not exist???
         return new RDXBehaviorTreeNode<>(id, rootNode);
      if (nodeType == AI2RNodeDefinition.class)
         return new RDXAI2RNode(id, rootNode);
      if (nodeType == ActionSequenceDefinition.class)
         return new RDXActionSequence(id, rootNode);
      if (nodeType == FallbackNodeDefinition.class)
         return new RDXFallbackNode(id, rootNode);
      if (nodeType == ConditionNodeDefinition.class)
         return new RDXConditionNode(id, rootNode);
      if (nodeType == GotoNodeDefinition.class)
         return new RDXGotoNode(id, rootNode);
      if (nodeType == CheckPointNodeDefinition.class)
         return new RDXCheckPointNode(id, rootNode);
      if (nodeType == DoorTraversalDefinition.class)
         return new RDXDoorTraversal(id, rootNode);
      if (nodeType == BuildingExplorationDefinition.class)
         return new RDXBuildingExploration(id, rootNode);
      if (nodeType == ChestOrientationActionDefinition.class)
         return new RDXChestOrientationAction(id, rootNode);
      if (nodeType == FootstepPlanActionDefinition.class)
         return new RDXFootstepPlanAction(id, rootNode);
      if (nodeType == HandPoseActionDefinition.class)
         return new RDXHandPoseAction(id, rootNode);
      if (nodeType == HandWrenchActionDefinition.class)
         return new RDXHandWrenchAction(id, rootNode);
      if (nodeType == ScrewPrimitiveActionDefinition.class)
         return new RDXScrewPrimitiveAction(id, rootNode);
      if (nodeType == PelvisHeightOrientationActionDefinition.class)
         return new RDXPelvisHeightOrientationAction(id, rootNode);
      if (nodeType == SakeHandCommandActionDefinition.class)
         return new RDXSakeHandCommandAction(id, rootNode);
      if (nodeType == WaitDurationActionDefinition.class)
         return new RDXWaitDurationAction(id, rootNode);
      if (nodeType == FootPoseActionDefinition.class)
         return new RDXFootPoseAction(id, rootNode);

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
