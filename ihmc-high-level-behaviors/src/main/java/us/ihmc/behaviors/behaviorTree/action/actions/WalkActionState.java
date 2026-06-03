package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.WalkActionFootstepStateMessage;
import behavior_msgs.WalkActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTStatusEnumField;
import us.ihmc.communication.crdt.CRDTStatusFootstepList;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.crdt.CRDTStatusPose3D;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.commons.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WalkActionState extends ActionNodeState<WalkActionDefinition>
{
   private final LatestTimestampModifiable stateDataSynchronizer;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<WalkActionFootstepState> manuallyPlacedFootsteps;
   private final CRDTBidirectionalRigidBodyTransform goalToParentTransform;
   private final SideDependentList<RigidBodyTransform> goalFootstepToGoalTransforms = new SideDependentList<>(() -> new RigidBodyTransform());
   private final DetachableReferenceFrame goalFrame;
   private ReferenceFrame parentFrame;
   private final CRDTStatusInteger totalNumberOfFootsteps;
   private final CRDTStatusInteger numberOfIncompleteFootsteps;
   private final SideDependentList<CRDTStatusSE3Trajectory> desiredFootPoses = new SideDependentList<>();
   private final SideDependentList<CRDTStatusPose3D> currentFootPoses = new SideDependentList<>();
   private final CRDTStatusEnumField<WalkActionExecutionState> executionState;
   private final CRDTStatusFootstepList previewFootsteps;

   public WalkActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new WalkActionDefinition(rootNode.getDefinition()), rootNode);

      // Prevents feedback loop where UI modifies definition fields and robot side updates state fields
      stateDataSynchronizer = new LatestTimestampModifiable(definition.getCRDTInfo());
      stateDataSynchronizer.setModifierName(definition.getName() + " state data:");

      goalToParentTransform = new CRDTBidirectionalRigidBodyTransform(stateDataSynchronizer);
      goalFrame = new DetachableReferenceFrame(scene::findFrameByName, goalToParentTransform.getValueReadOnly());
      manuallyPlacedFootsteps = new RecyclingArrayList<>(() ->
         new WalkActionFootstepState(scene,
                                             definition.getCRDTParentFrameName(),
                                             RecyclingArrayListTools.getUnsafe(definition.getManuallyPlacedFootsteps().getValueUnsafe(), numberOfAllocatedFootsteps++)));
      totalNumberOfFootsteps = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      for (RobotSide side : RobotSide.values)
      {
         desiredFootPoses.set(side, new CRDTStatusSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo));
         currentFootPoses.set(side, new CRDTStatusPose3D(ROS2ActorDesignation.ROBOT, crdtInfo));
      }
      executionState = new CRDTStatusEnumField<>(ROS2ActorDesignation.ROBOT, crdtInfo, WalkActionExecutionState.PLANNING_SUCCEEDED);
      previewFootsteps = new CRDTStatusFootstepList(ROS2ActorDesignation.ROBOT, crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();

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

   public void copyDefinitionToGoalFootstepToGoalTransform(RobotSide side)
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

   public void toMessage(WalkActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      stateDataSynchronizer.toMessage(message.getLatestModificationStateData());
      goalToParentTransform.toMessage(message.getGoalTransformToParent());
      message.setTotalNumberOfFootsteps((short) totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps((short) numberOfIncompleteFootsteps.toMessage());
      desiredFootPoses.get(RobotSide.LEFT).toMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).toMessage(message.getDesiredRightFootsteps());
      message.getCurrentLeftFootPose().set(currentFootPoses.get(RobotSide.LEFT).getValueReadOnly());
      message.getCurrentRightFootPose().set(currentFootPoses.get(RobotSide.RIGHT).getValueReadOnly());

      message.getFootsteps().clear();
      for (WalkActionFootstepState footstep : manuallyPlacedFootsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }

      message.setExecutionState(executionState.toMessage().toByte());
      previewFootsteps.toMessage(message.getPreviewFootsteps());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      stateDataSynchronizer.fromMessage(message.getLatestModificationStateData());
      goalToParentTransform.fromMessage(message.getGoalTransformToParent());
      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
      desiredFootPoses.get(RobotSide.LEFT).fromMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).fromMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).fromMessage(message.getCurrentLeftFootPose().getPose());
      currentFootPoses.get(RobotSide.RIGHT).fromMessage(message.getCurrentRightFootPose().getPose());

      manuallyPlacedFootsteps.clear();
      for (WalkActionFootstepStateMessage footstep : message.getFootsteps())
      {
         manuallyPlacedFootsteps.add().fromMessage(footstep);
      }

      executionState.fromMessage(WalkActionExecutionState.fromByte(message.getExecutionState()));
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
      return scene.containsFrame(definition.getParentFrameName()) && goalFrame.isChildOfWorld();
   }

   public ReferenceFrame getParentFrame()
   {
      return parentFrame;
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public RecyclingArrayList<WalkActionFootstepState> getManuallyPlacedFootsteps()
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

   public CRDTStatusEnumField<WalkActionExecutionState> getExecutionState()
   {
      return executionState;
   }

   public CRDTStatusFootstepList getPreviewFootsteps()
   {
      return previewFootsteps;
   }

   public ReferenceFrame getFrameByName(String frameName)
   {
      return scene.findFrameByName(frameName);
   }
}
