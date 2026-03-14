package us.ihmc.behaviors.behaviorTree.action;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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
      if (newAction instanceof ArmActionState handPoseAction)
      {
         // Set the new action to where the last one was for faster authoring
         handPoseAction.getDefinition().setSide(sideOfNewAction);
         handPoseAction.getDefinition().setPalmParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                             ArmActionState.class,
                                                                                             indexOfInsertion,
                                                                                             sideOfNewAction));
         handPoseAction.update();

         ArmActionState nextPreviousArmAction = findNextPreviousAction(actionSequence,
                                                                                 ArmActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousArmAction != null && nextPreviousArmAction.getPalmFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            handPoseAction.getDefinition().setPalmParentFrameName(nextPreviousArmAction.getDefinition().getPalmParentFrameName());
            handPoseAction.getDefinition().getPalmTransformToParent().getValueAndModify()
                          .set(nextPreviousArmAction.getDefinition().getPalmTransformToParent().getValueReadOnly());
         }
         else // set to current robot's hand pose in chest frame
         {
            handPoseAction.getDefinition().setPalmParentFrameName("Chest");
            syncedRobot.getReferenceFrames().getHandFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(handPoseAction.getDefinition().getPalmTransformToParent().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getChestFrame());
         }
         handPoseAction.update();
      }
      else if (newAction instanceof ScrewPrimitiveActionState screwPrimitiveAction)
      {
         screwPrimitiveAction.getDefinition().setSide(sideOfNewAction);
         screwPrimitiveAction.getDefinition()
                             .setObjectFrameName(findConvenientParentFrameName(actionSequence, ArmActionState.class, indexOfInsertion, sideOfNewAction));
         screwPrimitiveAction.update();
      }
      if (newAction instanceof LegActionState footPoseAction)
      {
         // Set the new action to where the last one was for faster authoring
         footPoseAction.getDefinition().setSide(sideOfNewAction);
         footPoseAction.getDefinition().setParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                         LegActionState.class,
                                                                                         indexOfInsertion,
                                                                                         sideOfNewAction));
         footPoseAction.update();

         LegActionState nextPreviousLegAction = findNextPreviousAction(actionSequence,
                                                                                 LegActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousLegAction != null && nextPreviousLegAction.getFootFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            footPoseAction.getDefinition().setParentFrameName(nextPreviousLegAction.getDefinition().getParentFrameName());
            footPoseAction.getDefinition().getFootToParentTransform().getValueAndModify()
                          .set(nextPreviousLegAction.getDefinition().getFootToParentTransform().getValueReadOnly());
         }
         else // set to current robot's foot pose in opposite (stance) foot frame
         {
            MovingReferenceFrame stanceFootFrame = syncedRobot.getReferenceFrames().getFootFrame(sideOfNewAction.getOppositeSide());
            footPoseAction.getDefinition().setParentFrameName(sideOfNewAction.getOppositeSide() + " Foot Sole");
            syncedRobot.getReferenceFrames().getFootFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(footPoseAction.getDefinition().getFootToParentTransform().getValueAndModify(),
                                                   stanceFootFrame);
         }
         footPoseAction.update();
      }
      else if (newAction instanceof SpineActionState chestOrientationAction)
      {
         SpineActionState nextPreviousAction = findNextPreviousAction(actionSequence, SpineActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getChestFrame().isChildOfWorld())
         {
            chestOrientationAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            chestOrientationAction.getDefinition().getChestToParentTransform().getValueAndModify()
                                  .set(nextPreviousAction.getDefinition().getChestToParentTransform().getValueReadOnly());
         }
         else
         {
            chestOrientationAction.getDefinition().setParentFrameName("Pelvis");
            syncedRobot.getReferenceFrames().getChestFrame()
                       .getTransformToDesiredFrame(chestOrientationAction.getDefinition().getChestToParentTransform().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getPelvisFrame());

         }
         chestOrientationAction.update();
      }
      else if (newAction instanceof PelvisActionState pelvisHeightPitchAction)
      {
         PelvisActionState nextPreviousAction = findNextPreviousAction(actionSequence, PelvisActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getPelvisFrame().isChildOfWorld())
         {
            pelvisHeightPitchAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            pelvisHeightPitchAction.getDefinition().getPelvisToParentTransform().getValueAndModify()
                                   .set(nextPreviousAction.getDefinition().getPelvisToParentTransform().getValueReadOnly());
         }
         else
         {
            pelvisHeightPitchAction.getDefinition().setParentFrameName("Walking");
            syncedRobot.getReferenceFrames().getPelvisFrame()
                       .getTransformToDesiredFrame(pelvisHeightPitchAction.getDefinition().getPelvisToParentTransform().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());

         }
         pelvisHeightPitchAction.update();
      }
      else if (newAction instanceof WalkActionState footstepPlanAction)
      {
         WalkActionState nextPreviousWalkAction = findNextPreviousAction(actionSequence, WalkActionState.class, indexOfInsertion, null);
         if (nextPreviousWalkAction != null)
         {
            footstepPlanAction.getDefinition().setParentFrameName(nextPreviousWalkAction.getDefinition().getParentFrameName());
         }
         else // set to current robot's pelvis pose
         {
            footstepPlanAction.getDefinition().setParentFrameName("Walking");
         }
         footstepPlanAction.update();
      }
      else if (newAction instanceof AbilityHandActionState abilityHandActionState)
      {
         abilityHandActionState.getDefinition().setSide(sideOfNewAction);
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

      if (nextPreviousAction instanceof WalkActionState footstepPlanAction)
      {
         return footstepPlanAction.getDefinition().getParentFrameName();
      }
      else if (nextPreviousAction instanceof ArmActionState handPoseAction)
      {
         return handPoseAction.getDefinition().getPalmParentFrameName();
      }

      return "Chest";
   }

   public static <T extends ActionNodeState<?>> T findNextPreviousAction(@Nullable BehaviorTreeRootNodeState actionSequence,
                                                                         Class<T> actionClass,
                                                                         int indexOfInsertion,
                                                                         @Nullable RobotSide side)
   {
      T previousAction = null;
      if (actionSequence != null)
      {
         previousAction = actionSequence.findNextPreviousLeaf(actionClass, indexOfInsertion, side);
      }
      return previousAction;
   }
}
