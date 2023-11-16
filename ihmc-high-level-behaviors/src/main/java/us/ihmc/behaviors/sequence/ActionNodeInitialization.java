package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.behaviors.sequence.actions.WalkActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

import javax.annotation.Nullable;

public class ActionNodeInitialization
{
   public static void initializeAction(ActionSequenceState actionSequence,
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
            syncedRobot.getReferenceFrames()
                       .getHandFrame(sideOfNewAction)
                       .getTransformToDesiredFrame(handPoseAction.getDefinition().getPalmTransformToParent().getValue(),
                                                   ReferenceFrame.getWorldFrame());
         }
         handPoseAction.update();
      }
   }

   /**
    * @return This used to find the most likely desired frame that a new action will
    *         be specified in, by traversing the sequence backwards and finding the
    *         first action that is specified in a frame, and returning that frame.
    *         This helps the authoring process by initializing new actions with
    *         spatially consistent values.
    */
   private static String findConvenientParentFrameName(ActionSequenceState actionSequence,
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

   private static <T extends ActionNodeState<?>> T findNextPreviousAction(ActionSequenceState actionSequence,
                                                                          Class<T> actionClass,
                                                                          int indexOfInsertion,
                                                                          @Nullable RobotSide side)
   {
      T previousAction = null;

      for (int i = indexOfInsertion - 1; i >= 0; i--)
      {
         BehaviorTreeNodeState<?> action = actionSequence.getChildren().get(i);
         if (actionClass.isInstance(action))
         {
            boolean match = side == null;
            match |= action.getDefinition() instanceof SidedObject sidedAction && sidedAction.getSide() == side;

            if (match)
            {
               previousAction = actionClass.cast(action);
            }
         }
      }
      return previousAction;
   }
}
