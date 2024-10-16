package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateBuilder;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.sequence.ActionNodeInitialization;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.actions.*;
import us.ihmc.rdx.ui.behavior.behaviors.RDXAI2RNode;
import us.ihmc.rdx.ui.behavior.behaviors.RDXDoorTraversal;
import us.ihmc.rdx.ui.behavior.behaviors.RDXTrashCanInteraction;
import us.ihmc.rdx.ui.behavior.behaviors.RDXBuildingExploration;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXBehaviorTreeNodeBuilder implements BehaviorTreeNodeStateBuilder
{
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
   public RDXBehaviorTreeNode<?, ?> createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      // Control nodes:
      if (nodeType == BehaviorTreeRootNodeDefinition.class)
      {
         return new RDXBehaviorTreeRootNode(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == BehaviorTreeNodeDefinition.class)
      {
         return new RDXBehaviorTreeNode<>(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == AI2RNodeDefinition.class)
      {
         return new RDXAI2RNode(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == ActionSequenceDefinition.class)
      {
         return new RDXActionSequence(id, crdtInfo, saveFileDirectory);
      }
      if (nodeType == DoorTraversalDefinition.class)
      {
         return new RDXDoorTraversal(id, crdtInfo, saveFileDirectory, syncedRobot);
      }
      if (nodeType == TrashCanInteractionDefinition.class)
      {
         return new RDXTrashCanInteraction(id, crdtInfo, saveFileDirectory, syncedRobot);
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
