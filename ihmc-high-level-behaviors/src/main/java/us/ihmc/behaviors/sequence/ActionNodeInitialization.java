package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

import javax.annotation.Nullable;

public class ActionNodeInitialization
{
   public static void initializeAction(@Nullable ActionSequenceState actionSequence,
                                       ActionNodeState<?> newAction,
                                       int indexOfInsertion,
                                       @Nullable RobotSide sideOfNewAction,
                                       ROS2SyncedRobotModel syncedRobot)
   {
      if (newAction instanceof WalkActionState walkAction)
      {
         MovingReferenceFrame parentFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
         walkAction.getDefinition().setParentFrameName(parentFrame.getName());
         walkAction.getState().update();
      }
      else if (newAction instanceof HandPoseActionState handPoseAction)
      {
         // Set the new action to where the last one was for faster authoring
         handPoseAction.getDefinition().setSide(sideOfNewAction);
         handPoseAction.getDefinition().setPalmParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                             handPoseAction,
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
      else if (newAction instanceof PelvisHeightPitchActionState pelvisHeightPitchAction)
      {
         PelvisHeightPitchActionState nextPreviousAction = findNextPreviousAction(actionSequence, PelvisHeightPitchActionState.class, indexOfInsertion, null);
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
            footstepPlanAction.getDefinition().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
         }
         footstepPlanAction.update();
      }
   }

   /**
    * @return This used to find the most likely desired frame that a new action will
    *         be specified in, by traversing the sequence backwards and finding the
    *         first action that is specified in a frame, and returning that frame.
    *         This helps the authoring process by initializing new actions with
    *         spatially consistent values.
    */
   private static String findConvenientParentFrameName(@Nullable ActionSequenceState actionSequence,
                                                       ActionNodeState<?> action,
                                                       int indexOfInsertion,
                                                       @Nullable RobotSide side)
   {
      ActionNodeState<?> nextPreviousAction = findNextPreviousAction(actionSequence, action.getClass(), indexOfInsertion, side);

      if (nextPreviousAction instanceof FootstepPlanActionState footstepPlanAction)
      {
         return footstepPlanAction.getDefinition().getParentFrameName();
      }
      else if (nextPreviousAction instanceof HandPoseActionState handPoseAction)
      {
         return handPoseAction.getDefinition().getPalmParentFrameName();
      }
      else if (nextPreviousAction instanceof WalkActionState walkAction)
      {
         return walkAction.getDefinition().getParentFrameName();
      }

      return ReferenceFrame.getWorldFrame().getName();
   }

   public static <T extends ActionNodeState<?>> T findNextPreviousAction(@Nullable ActionSequenceState actionSequence,
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
