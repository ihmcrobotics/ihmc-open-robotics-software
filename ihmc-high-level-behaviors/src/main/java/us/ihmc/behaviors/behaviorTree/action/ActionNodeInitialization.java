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
      if (newAction instanceof ArmActionState armAction)
      {
         // Set the new action to where the last one was for faster authoring
         armAction.getDefinition().setSide(sideOfNewAction);
         armAction.getDefinition().setPalmParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                             ArmActionState.class,
                                                                                             indexOfInsertion,
                                                                                             sideOfNewAction));
         armAction.update();

         ArmActionState nextPreviousArmAction = findNextPreviousAction(actionSequence,
                                                                                 ArmActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousArmAction != null && nextPreviousArmAction.getPalmFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            armAction.getDefinition().setPalmParentFrameName(nextPreviousArmAction.getDefinition().getPalmParentFrameName());
            armAction.getDefinition().getPalmTransformToParent().getValueAndModify()
                          .set(nextPreviousArmAction.getDefinition().getPalmTransformToParent().getValueReadOnly());
         }
         else // set to current robot's hand pose in chest frame
         {
            armAction.getDefinition().setPalmParentFrameName("Chest");
            syncedRobot.getReferenceFrames().getHandFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(armAction.getDefinition().getPalmTransformToParent().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getChestFrame());
         }
         armAction.update();
      }
      if (newAction instanceof LegActionState legAction)
      {
         // Set the new action to where the last one was for faster authoring
         legAction.getDefinition().setSide(sideOfNewAction);
         legAction.getDefinition().setParentFrameName(findConvenientParentFrameName(actionSequence,
                                                                                         LegActionState.class,
                                                                                         indexOfInsertion,
                                                                                         sideOfNewAction));
         legAction.update();

         LegActionState nextPreviousLegAction = findNextPreviousAction(actionSequence,
                                                                                 LegActionState.class,
                                                                                 indexOfInsertion,
                                                                                 sideOfNewAction);
         if (nextPreviousLegAction != null && nextPreviousLegAction.getFootFrame().isChildOfWorld())
         {
            // Set pose to previous hand pose
            legAction.getDefinition().setParentFrameName(nextPreviousLegAction.getDefinition().getParentFrameName());
            legAction.getDefinition().getFootToParentTransform().getValueAndModify()
                          .set(nextPreviousLegAction.getDefinition().getFootToParentTransform().getValueReadOnly());
         }
         else // set to current robot's foot pose in opposite (stance) foot frame
         {
            MovingReferenceFrame stanceFootFrame = syncedRobot.getReferenceFrames().getFootFrame(sideOfNewAction.getOppositeSide());
            legAction.getDefinition().setParentFrameName(sideOfNewAction.getOppositeSide() + " Foot Sole");
            syncedRobot.getReferenceFrames().getFootFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(legAction.getDefinition().getFootToParentTransform().getValueAndModify(),
                                                   stanceFootFrame);
         }
         legAction.update();
      }
      else if (newAction instanceof SpineActionState spineAction)
      {
         SpineActionState nextPreviousAction = findNextPreviousAction(actionSequence, SpineActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getChestFrame().isChildOfWorld())
         {
            spineAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            spineAction.getDefinition().getChestToParentTransform().getValueAndModify()
                                  .set(nextPreviousAction.getDefinition().getChestToParentTransform().getValueReadOnly());
         }
         else
         {
            spineAction.getDefinition().setParentFrameName("Pelvis");
            syncedRobot.getReferenceFrames().getChestFrame()
                       .getTransformToDesiredFrame(spineAction.getDefinition().getChestToParentTransform().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getPelvisFrame());

         }
         spineAction.update();
      }
      else if (newAction instanceof PelvisActionState pelvisAction)
      {
         PelvisActionState nextPreviousAction = findNextPreviousAction(actionSequence, PelvisActionState.class, indexOfInsertion, null);
         if (nextPreviousAction != null && nextPreviousAction.getPelvisFrame().isChildOfWorld())
         {
            pelvisAction.getDefinition().setParentFrameName(nextPreviousAction.getDefinition().getParentFrameName());
            pelvisAction.getDefinition().getPelvisToParentTransform().getValueAndModify()
                                   .set(nextPreviousAction.getDefinition().getPelvisToParentTransform().getValueReadOnly());
         }
         else
         {
            pelvisAction.getDefinition().setParentFrameName("Walking");
            syncedRobot.getReferenceFrames().getPelvisFrame()
                       .getTransformToDesiredFrame(pelvisAction.getDefinition().getPelvisToParentTransform().getValueAndModify(),
                                                   syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());

         }
         pelvisAction.update();
      }
      else if (newAction instanceof WalkActionState walkAction)
      {
         WalkActionState nextPreviousWalkAction = findNextPreviousAction(actionSequence, WalkActionState.class, indexOfInsertion, null);
         if (nextPreviousWalkAction != null)
         {
            walkAction.getDefinition().setParentFrameName(nextPreviousWalkAction.getDefinition().getParentFrameName());
         }
         else // set to current robot's pelvis pose
         {
            walkAction.getDefinition().setParentFrameName("Walking");
         }
         walkAction.update();
      }
      else if (newAction instanceof AbilityHandActionState abilityHandActionState)
      {
         abilityHandActionState.getDefinition().setSide(sideOfNewAction);
      }
      else if (newAction instanceof EZGripperActionState ezGripperActionState)
      {
         ezGripperActionState.getDefinition().setSide(sideOfNewAction);
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

      if (nextPreviousAction instanceof WalkActionState walkAction)
      {
         return walkAction.getDefinition().getParentFrameName();
      }
      else if (nextPreviousAction instanceof ArmActionState armAction)
      {
         return armAction.getDefinition().getPalmParentFrameName();
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
