package us.ihmc.rdx.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeBuilder;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeInitialization;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.behaviorTree.actions.*;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.behaviorTree.control.RDXAI2RNode;
import us.ihmc.rdx.behaviorTree.control.RDXDoorTraversal;
import us.ihmc.rdx.behaviorTree.control.RDXBuildingExploration;
import us.ihmc.rdx.behaviorTree.condition.RDXConditionNode;
import us.ihmc.rdx.behaviorTree.control.RDXGotoNode;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;
import us.ihmc.rdx.behaviorTree.control.RDXActionSequence;
import us.ihmc.rdx.behaviorTree.control.RDXFallbackNode;
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
   private final RobotCollisionModel selectionCollisionModel;
   private final RDXBaseUI baseUI;
   private final RDX3DPanel panel3D;
   private final ReferenceFrameLibrary referenceFrameLibrary;

   public RDXBehaviorTreeNodeBuilder(DRCRobotModel robotModel,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel selectionCollisionModel,
                                     RDXBaseUI baseUI,
                                     RDX3DPanel panel3D,
                                     ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
      this.referenceFrameLibrary = referenceFrameLibrary;
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
      return new RDXBehaviorTreeRootNode(id, crdtInfo, saveFileDirectory);
   }

   @Override
   public RDXBehaviorTreeNode<?, ?> createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>> rootNodeType)
   {
      RDXBehaviorTreeRootNode rootNode = (RDXBehaviorTreeRootNode) rootNodeType;

      // Control nodes:
      if (nodeType == BehaviorTreeNodeDefinition.class) // TODO: Should not exist???
      {
         return new RDXBehaviorTreeNode<>(id, rootNode);
      }
      if (nodeType == AI2RNodeDefinition.class)
      {
         return new RDXAI2RNode(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == ActionSequenceDefinition.class)
      {
         return new RDXActionSequence(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == FallbackNodeDefinition.class)
      {
         return new RDXFallbackNode(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == ConditionNodeDefinition.class)
      {
         return new RDXConditionNode(id, crdtInfo, saveFileDirectory, referenceFrameLibrary);
      }
      if (nodeType == GotoNodeDefinition.class)
      {
         return new RDXGotoNode(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == CheckPointNodeDefinition.class)
      {
         return new RDXCheckPointNode(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == DoorTraversalDefinition.class)
      {
         return new RDXDoorTraversal(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == BuildingExplorationDefinition.class)
      {
         return new RDXBuildingExploration(id, crdtInfo, saveFileDirectory, syncedRobot);
      }

      // Actions:
      if (nodeType == ChestOrientationActionDefinition.class)
      {
         return new RDXChestOrientationAction(id,
                                              crdtInfo,
                                              saveFileDirectory,
                                              panel3D,
                                              robotModel,
                                              syncedRobot.getFullRobotModel(),
                                              selectionCollisionModel,
                                              referenceFrameLibrary);
      }
      if (nodeType == FootstepPlanActionDefinition.class)
      {
         return new RDXFootstepPlanAction(id,
                                          crdtInfo,
                                          saveFileDirectory,
                                          baseUI,
                                          robotModel,
                                          syncedRobot,
                                          referenceFrameLibrary);
      }
      if (nodeType == HandPoseActionDefinition.class)
      {
         return new RDXHandPoseAction(id,
                                      crdtInfo,
                                      saveFileDirectory,
                                      panel3D,
                                      robotModel,
                                      syncedRobot,
                                      selectionCollisionModel,
                                      referenceFrameLibrary);
      }
      if (nodeType == HandWrenchActionDefinition.class)
      {
         return new RDXHandWrenchAction(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == ScrewPrimitiveActionDefinition.class)
      {
         return new RDXScrewPrimitiveAction(id, crdtInfo, saveFileDirectory, panel3D, referenceFrameLibrary, syncedRobot);
      }
      if (nodeType == PelvisHeightOrientationActionDefinition.class)
      {
         return new RDXPelvisHeightOrientationAction(id,
                                                     crdtInfo,
                                                     saveFileDirectory,
                                                     panel3D,
                                                     robotModel,
                                                     syncedRobot.getFullRobotModel(),
                                                     selectionCollisionModel,
                                                     referenceFrameLibrary);
      }
      if (nodeType == SakeHandCommandActionDefinition.class)
      {
         return new RDXSakeHandCommandAction(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == WaitDurationActionDefinition.class)
      {
         return new RDXWaitDurationAction(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == FootPoseActionDefinition.class)
      {
         return new RDXFootPoseAction(id,
                                      crdtInfo,
                                      saveFileDirectory,
                                      panel3D,
                                      robotModel,
                                      syncedRobot.getFullRobotModel(),
                                      selectionCollisionModel,
                                      referenceFrameLibrary);
      }
      else
      {
         return null;
      }
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
