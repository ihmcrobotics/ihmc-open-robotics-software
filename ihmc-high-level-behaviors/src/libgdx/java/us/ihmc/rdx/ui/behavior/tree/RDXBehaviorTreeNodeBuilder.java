package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateBuilder;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionNodeInitialization;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.actions.*;
import us.ihmc.rdx.ui.behavior.behaviors.RDXDoorTraversal;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
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
   private final FootstepPlannerParametersBasics footstepPlannerParametersBasics;

   public RDXBehaviorTreeNodeBuilder(DRCRobotModel robotModel,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel selectionCollisionModel,
                                     RDXBaseUI baseUI,
                                     RDX3DPanel panel3D,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     FootstepPlannerParametersBasics footstepPlannerParametersBasics)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.footstepPlannerParametersBasics = footstepPlannerParametersBasics;
   }

   @Override
   public RDXBehaviorTreeNode<?, ?> createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      BehaviorTreeDefinitionRegistry nodeEnum = BehaviorTreeDefinitionRegistry.getTypeFromClass(nodeType);
      if (nodeEnum == null)
      {
         throw new RuntimeException("No known node type for class " + nodeType.getSimpleName());
      }

      return switch (nodeEnum)
      {
         // Control nodes:
         case BASIC_NODE -> new RDXBehaviorTreeNode<>(id, crdtInfo, saveFileDirectory);
         case ACTION_SEQUENCE -> new RDXActionSequence(id, crdtInfo, saveFileDirectory);
         case DOOR_TRAVERSAL -> new RDXDoorTraversal(id, crdtInfo, saveFileDirectory, syncedRobot);
         // Actions:
         case CHEST_ORIENTATION_ACTION -> new RDXChestOrientationAction(id,
                                                                        crdtInfo,
                                                                        saveFileDirectory,
                                                                        panel3D,
                                                                        robotModel,
                                                                        syncedRobot.getFullRobotModel(),
                                                                        selectionCollisionModel,
                                                                        referenceFrameLibrary);
         case FOOTSTEP_PLAN_ACTION -> new RDXFootstepPlanAction(id,
                                                                crdtInfo,
                                                                saveFileDirectory,
                                                                baseUI,
                                                                robotModel,
                                                                syncedRobot,
                                                                referenceFrameLibrary,
                                                                footstepPlannerParametersBasics);
         case HAND_POSE_ACTION ->
               new RDXHandPoseAction(id, crdtInfo, saveFileDirectory, panel3D, robotModel, syncedRobot, selectionCollisionModel, referenceFrameLibrary);
         case HAND_WRENCH_ACTION -> new RDXHandWrenchAction(id, crdtInfo, saveFileDirectory);
         case SCREW_PRIMITIVE_ACTION -> new RDXScrewPrimitiveAction(id, crdtInfo, saveFileDirectory, panel3D, referenceFrameLibrary, syncedRobot);
         case PELVIS_HEIGHT_PITCH_ACTION -> new RDXPelvisHeightPitchAction(id,
                                                                           crdtInfo,
                                                                           saveFileDirectory,
                                                                           panel3D,
                                                                           robotModel,
                                                                           syncedRobot.getFullRobotModel(),
                                                                           selectionCollisionModel,
                                                                           referenceFrameLibrary);
         case SAKE_HAND_COMMAND_ACTION -> new RDXSakeHandCommandAction(id, crdtInfo, saveFileDirectory);
         case WAIT_DURATION_ACTION -> new RDXWaitDurationAction(id, crdtInfo, saveFileDirectory);
         case KICK_DOOR_ACTION ->
               new RDXKickDoorAction(id, crdtInfo, saveFileDirectory, baseUI, robotModel, syncedRobot, referenceFrameLibrary, footstepPlannerParametersBasics);
         case KICK_DOOR_APPROACH_ACTION -> new RDXKickDoorApproachPlanAction(id,
                                                                             crdtInfo,
                                                                             saveFileDirectory,
                                                                             baseUI,
                                                                             robotModel,
                                                                             referenceFrameLibrary);
         default -> throw new RuntimeException("No return type defined for class " + nodeType.getSimpleName());
      };
   }



   // This method is in this class because we have a syncedRobot here.
   public void initializeActionNode(@Nullable RDXActionSequence actionSequence,
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
