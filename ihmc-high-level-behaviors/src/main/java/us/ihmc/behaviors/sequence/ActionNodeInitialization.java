package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;

public class ActionNodeInitialization
{
   public static void initializeAction(@Nullable BehaviorTreeRootNodeState actionSequence,
                                       ActionNodeState<?> newAction,
                                       int indexOfInsertion,
                                       @Nullable RobotSide sideOfNewAction,
                                       ROS2SyncedRobotModel syncedRobot)
   {
      if (newAction instanceof HandPoseActionState handPoseAction)
      {
         // Set the new action to where the last one was for faster authoring
         handPoseAction.getDefinition().setSide(sideOfNewAction);
         handPoseAction.getDefinition().setPalmParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                             HandPoseActionState.class,
                                                                                             indexOfInsertion,
                                                                                             sideOfNewAction));
         handPoseAction.getState().update();

         HandPoseActionState nextPreviousHandPoseAction = findNextPreviousAction(actionSequence,
                                                                                 HandPoseActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousHandPoseAction != null && nextPreviousHandPoseAction.getPalmFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            handPoseAction.getDefinition().setPalmParentFrameName(nextPreviousHandPoseAction.getDefinition().getPalmParentFrameName());
            handPoseAction.getDefinition().getPalmTransformToParent().getValue()
                          .set(nextPreviousHandPoseAction.getDefinition().getPalmTransformToParent().getValueReadOnly());
         }
         else // set to current robot's hand pose
         {
            handPoseAction.getDefinition().setPalmParentFrameName(ReferenceFrame.getWorldFrame().getName());
            syncedRobot.getReferenceFrames().getHandFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(handPoseAction.getDefinition().getPalmTransformToParent().getValue(),
                                                   ReferenceFrame.getWorldFrame());
         }
         handPoseAction.update();
      }
      else if (newAction instanceof ScrewPrimitiveActionState screwPrimitiveAction)
      {
         screwPrimitiveAction.getDefinition().setSide(sideOfNewAction);
         screwPrimitiveAction.getDefinition()
                             .setObjectFrameName(findConvenientParentFrameName(actionSequence, HandPoseActionState.class, indexOfInsertion, sideOfNewAction));
         screwPrimitiveAction.getState().update();
      }
      else if ( newAction instanceof KickDoorApproachPlanActionState kickDoorApproachPlanAction)
      {
         KickDoorApproachPlanActionState nextPreviousFootstepPlanAction = findNextPreviousAction(actionSequence, KickDoorApproachPlanActionState.class, indexOfInsertion, null);
         if (nextPreviousFootstepPlanAction != null)
         {
            kickDoorApproachPlanAction.getDefinition().setParentFrameName(nextPreviousFootstepPlanAction.getDefinition().getParentFrameName());
         }
         else
         {
            kickDoorApproachPlanAction.getDefinition().setParentFrameName(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getName());
         }

         kickDoorApproachPlanAction.getDefinition().setSide(sideOfNewAction);
         kickDoorApproachPlanAction.update();
      }
      else if ( newAction instanceof KickDoorActionState kickDoorAction)
      {
         KickDoorActionState nextPreviousFootstepPlanAction = findNextPreviousAction(actionSequence, KickDoorActionState.class, indexOfInsertion, null);
         if (nextPreviousFootstepPlanAction != null)
         {
            kickDoorAction.getDefinition().setParentFrameName(nextPreviousFootstepPlanAction.getDefinition().getParentFrameName());
         }
         else
         {
            kickDoorAction.getDefinition().setParentFrameName(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getName());
         }

         kickDoorAction.getDefinition().setSide(sideOfNewAction);
         kickDoorAction.update();
      }
      if (newAction instanceof FootPoseActionState footPoseAction)
      {
         // Set the new action to where the last one was for faster authoring
         footPoseAction.getDefinition().setSide(sideOfNewAction);
         footPoseAction.getDefinition().setParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                         FootPoseActionState.class,
                                                                                         indexOfInsertion,
                                                                                         sideOfNewAction));
         footPoseAction.getState().update();

         FootPoseActionState nextPreviousFootPoseAction = findNextPreviousAction(actionSequence,
                                                                                 FootPoseActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousFootPoseAction != null && nextPreviousFootPoseAction.getFootFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            footPoseAction.getDefinition().setParentFrameName(nextPreviousFootPoseAction.getDefinition().getParentFrameName());
            footPoseAction.getDefinition().getFootToParentTransform().getValue()
                          .set(nextPreviousFootPoseAction.getDefinition().getFootToParentTransform().getValueReadOnly());
         }
         else // set to current robot's hand pose
         {
            footPoseAction.getDefinition().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
            syncedRobot.getReferenceFrames().getFootFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(footPoseAction.getDefinition().getFootToParentTransform().getValue(),
                                                   ReferenceFrame.getWorldFrame());
         }
         footPoseAction.update();
      }
      else if (newAction instanceof ChestOrientationActionState chestOrientationAction)
      {
         ChestOrientationActionState nextPreviousAction = findNextPreviousAction(actionSequence, ChestOrientationActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getChestFrame().isChildOfWorld())
         {
            chestOrientationAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            chestOrientationAction.getDefinition().getChestToParentTransform().getValue()
                                  .set(nextPreviousAction.getDefinition().getChestToParentTransform().getValueReadOnly());
         }
         else
         {
            chestOrientationAction.getDefinition().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
            syncedRobot.getReferenceFrames().getChestFrame()
                       .getTransformToDesiredFrame(chestOrientationAction.getDefinition().getChestToParentTransform().getValue(),
                                                   ReferenceFrame.getWorldFrame());

         }
         chestOrientationAction.update();
      }
      else if (newAction instanceof PelvisHeightOrientationActionState pelvisHeightPitchAction)
      {
         PelvisHeightOrientationActionState nextPreviousAction = findNextPreviousAction(actionSequence, PelvisHeightOrientationActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getPelvisFrame().isChildOfWorld())
         {
            pelvisHeightPitchAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            pelvisHeightPitchAction.getDefinition().getPelvisToParentTransform().getValue()
                                   .set(nextPreviousAction.getDefinition().getPelvisToParentTransform().getValueReadOnly());
         }
         else
         {
            pelvisHeightPitchAction.getDefinition().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
            syncedRobot.getReferenceFrames().getPelvisFrame()
                       .getTransformToDesiredFrame(pelvisHeightPitchAction.getDefinition().getPelvisToParentTransform().getValue(),
                                                   ReferenceFrame.getWorldFrame());

         }
         pelvisHeightPitchAction.update();
      }
      else if (newAction instanceof FootstepPlanActionState footstepPlanAction)
      {
         FootstepPlanActionState nextPreviousFootstepPlanAction = findNextPreviousAction(actionSequence, FootstepPlanActionState.class, indexOfInsertion, null);
         if (nextPreviousFootstepPlanAction != null)
         {
            footstepPlanAction.getDefinition().setParentFrameName(nextPreviousFootstepPlanAction.getDefinition().getParentFrameName());
         }
         else // set to current robot's pelvis pose
         {
            footstepPlanAction.getDefinition().setParentFrameName(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getName());
         }
         footstepPlanAction.update();
      }
      else if (newAction instanceof SakeHandCommandActionState sakeHandCommandActionState)
      {
         sakeHandCommandActionState.getDefinition().setSide(sideOfNewAction);
      }
   }

   /**
    * @return This used to find the most likely desired frame that a new action will
    *         be specified in, by traversing the sequence backwards and finding the
    *         first action that is specified in a frame, and returning that frame.
    *         This helps the authoring process by initializing new actions with
    *         spatially consistent values.
    */
   private static String findConvenientParentFrameName(@Nullable BehaviorTreeRootNodeState actionSequence,
                                                       Class<? extends ActionNodeState<?>> actionClass,
                                                       int indexOfInsertion,
                                                       @Nullable RobotSide side)
   {
      ActionNodeState<?> nextPreviousAction = findNextPreviousAction(actionSequence, actionClass, indexOfInsertion, side);

      if (nextPreviousAction instanceof FootstepPlanActionState footstepPlanAction)
      {
         return footstepPlanAction.getDefinition().getParentFrameName();
      }
      else if (nextPreviousAction instanceof HandPoseActionState handPoseAction)
      {
         return handPoseAction.getDefinition().getPalmParentFrameName();
      }

      return ReferenceFrame.getWorldFrame().getName();
   }

   public static <T extends ActionNodeState<?>> T findNextPreviousAction(@Nullable BehaviorTreeRootNodeState actionSequence,
                                                                         Class<T> actionClass,
                                                                         int indexOfInsertion,
                                                                         @Nullable RobotSide side)
   {
      T previousAction = null;
      if (actionSequence != null)
      {
         previousAction = actionSequence.findNextPreviousAction(actionClass, indexOfInsertion, side);
      }
      return previousAction;
   }
}
