package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import behavior_msgs.msg.dds.FootstepPlanActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusEnumField;
import us.ihmc.communication.crdt.CRDTStatusFootstepList;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.crdt.CRDTStatusPose3D;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionState extends ActionNodeState<FootstepPlanActionDefinition>
{
   private final FootstepPlanActionDefinition definition;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<FootstepPlanActionFootstepState> manuallyPlacedFootsteps;
   private final CRDTBidirectionalRigidBodyTransform goalToParentTransform;
   private final SideDependentList<RigidBodyTransform> goalFootstepToGoalTransforms = new SideDependentList<>(() -> new RigidBodyTransform());
   private final DetachableReferenceFrame goalFrame;
   private ReferenceFrame parentFrame;
   private final CRDTStatusInteger totalNumberOfFootsteps;
   private final CRDTStatusInteger numberOfIncompleteFootsteps;
   private final SideDependentList<CRDTStatusSE3Trajectory> desiredFootPoses = new SideDependentList<>();
   private final SideDependentList<CRDTStatusPose3D> currentFootPoses = new SideDependentList<>();
   private final CRDTStatusEnumField<FootstepPlanActionExecutionState> executionState;
   private final CRDTStatusFootstepList previewFootsteps;

   public FootstepPlanActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new FootstepPlanActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      this.referenceFrameLibrary = referenceFrameLibrary;

      goalToParentTransform = new CRDTBidirectionalRigidBodyTransform(definition);
      goalFrame = new DetachableReferenceFrame(referenceFrameLibrary, goalToParentTransform.getValueReadOnly());
      manuallyPlacedFootsteps = new RecyclingArrayList<>(() ->
         new FootstepPlanActionFootstepState(referenceFrameLibrary,
                                             definition.getCRDTParentFrameName(),
                                             RecyclingArrayListTools.getUnsafe(definition.getManuallyPlacedFootsteps().getValueUnsafe(), numberOfAllocatedFootsteps++)));
      totalNumberOfFootsteps = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      for (RobotSide side : RobotSide.values)
      {
         desiredFootPoses.set(side, new CRDTStatusSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo));
         currentFootPoses.set(side, new CRDTStatusPose3D(ROS2ActorDesignation.ROBOT, crdtInfo));
      }
      executionState = new CRDTStatusEnumField<>(ROS2ActorDesignation.ROBOT, crdtInfo, FootstepPlanActionExecutionState.PLANNING_SUCCEEDED);
      previewFootsteps = new CRDTStatusFootstepList(ROS2ActorDesignation.ROBOT, crdtInfo);
   }

   @Override
   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         goalFootstepToGoalTransforms.get(side).getTranslation().setZ(0.0);
         goalFootstepToGoalTransforms.get(side).getRotation().setYawPitchRoll(goalFootstepToGoalTransforms.get(side).getRotation().getYaw(), 0.0, 0.0);
      }

      goalFrame.update(definition.getParentFrameName());
      parentFrame = goalFrame.getReferenceFrame().getParent();

      RecyclingArrayListTools.synchronizeSize(manuallyPlacedFootsteps, definition.getManuallyPlacedFootsteps().getSize());

      for (int i = 0; i < manuallyPlacedFootsteps.size(); i++)
      {
         manuallyPlacedFootsteps.get(i).setIndex(i);
         manuallyPlacedFootsteps.get(i).update();
      }
   }

   public void copyDefinitionToGoalFoostepToGoalTransform(RobotSide side)
   {
      goalFootstepToGoalTransforms.get(side).setToZero();
      goalFootstepToGoalTransforms.get(side).getTranslation().setX(definition.getGoalFootstepToGoalX(side).getValue());
      goalFootstepToGoalTransforms.get(side).getTranslation().setY(definition.getGoalFootstepToGoalY(side).getValue());
      goalFootstepToGoalTransforms.get(side).getRotation().setToYawOrientation(definition.getGoalFootstepToGoalYaw(side).getValue());
   }

   public void copyGoalFootstepToGoalTransformToDefinition(RobotSide side)
   {
      definition.getGoalFootstepToGoalX(side).setValue(goalFootstepToGoalTransforms.get(side).getTranslation().getX());
      definition.getGoalFootstepToGoalY(side).setValue(goalFootstepToGoalTransforms.get(side).getTranslation().getY());
      definition.getGoalFootstepToGoalYaw(side).setValue(goalFootstepToGoalTransforms.get(side).getRotation().getYaw());
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= totalNumberOfFootsteps.pollHasStatus();
      hasStatus |= numberOfIncompleteFootsteps.pollHasStatus();
      for (RobotSide side : RobotSide.values)
      {
         hasStatus |= desiredFootPoses.get(side).pollHasStatus();
         hasStatus |= currentFootPoses.get(side).pollHasStatus();
      }
      hasStatus |= executionState.pollHasStatus();
      hasStatus |= previewFootsteps.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(FootstepPlanActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalToParentTransform.toMessage(message.getGoalTransformToParent());
      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps.toMessage());
      desiredFootPoses.get(RobotSide.LEFT).toMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).toMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).toMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).toMessage(message.getCurrentRightFootPose());

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : manuallyPlacedFootsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }

      message.setExecutionState(executionState.toMessage().toByte());
      previewFootsteps.toMessage(message.getPreviewFootsteps());
   }

   public void fromMessage(FootstepPlanActionStateMessage message)
   {
      super.fromMessage(message.getState());

      definition.fromMessage(message.getDefinition());

      goalToParentTransform.fromMessage(message.getGoalTransformToParent());
      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
      desiredFootPoses.get(RobotSide.LEFT).fromMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).fromMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).fromMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).fromMessage(message.getCurrentRightFootPose());

      manuallyPlacedFootsteps.clear();
      for (FootstepPlanActionFootstepStateMessage footstep : message.getFootsteps())
      {
         manuallyPlacedFootsteps.add().fromMessage(footstep);
      }

      executionState.fromMessage(FootstepPlanActionExecutionState.fromByte(message.getExecutionState()));
      previewFootsteps.fromMessage(message.getPreviewFootsteps());
   }

   public CRDTBidirectionalRigidBodyTransform getGoalToParentTransform()
   {
      return goalToParentTransform;
   }

   public RigidBodyTransform getGoalFootstepToGoalTransform(RobotSide side)
   {
      return goalFootstepToGoalTransforms.get(side);
   }

   public boolean areFramesInWorld()
   {
      return referenceFrameLibrary.containsFrame(definition.getParentFrameName()) && goalFrame.isChildOfWorld();
   }

   public ReferenceFrame getParentFrame()
   {
      return parentFrame;
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getManuallyPlacedFootsteps()
   {
      return manuallyPlacedFootsteps;
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps.getValue();
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps.setValue(totalNumberOfFootsteps);
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps.getValue();
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps.setValue(numberOfIncompleteFootsteps);
   }

   public SideDependentList<CRDTStatusSE3Trajectory> getDesiredFootPoses()
   {
      return desiredFootPoses;
   }

   public SideDependentList<CRDTStatusPose3D> getCurrentFootPoses()
   {
      return currentFootPoses;
   }

   public CRDTStatusEnumField<FootstepPlanActionExecutionState> getExecutionState()
   {
      return executionState;
   }

   public CRDTStatusFootstepList getPreviewFootsteps()
   {
      return previewFootsteps;
   }
}
