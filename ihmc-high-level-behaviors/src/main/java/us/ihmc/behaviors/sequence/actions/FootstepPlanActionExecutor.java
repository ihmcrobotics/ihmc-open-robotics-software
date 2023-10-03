package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

import java.util.UUID;

public class FootstepPlanActionExecutor implements BehaviorActionExecutor
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FootstepPlanActionState state = new FootstepPlanActionState();
   private final FootstepPlanActionDefinition definition = state.getDefinition();
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final WalkingControllerParameters walkingControllerParameters;
   private int actionIndex;
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final SideDependentList<BehaviorActionCompletionCalculator> completionCalculator = new SideDependentList<>(BehaviorActionCompletionCalculator::new);
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;

   public FootstepPlanActionExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     WalkingFootstepTracker footstepTracker,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     WalkingControllerParameters walkingControllerParameters)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      footstepPlanToExecute.clear();
      for (FootstepActionState footstep : state.getFootsteps())
      {
         solePose.setIncludingFrame(footstep.getSoleFrame().getReferenceFrame().getParent(),
                                    footstep.getDefinition().getSoleToPlanFrameTransform());
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         footstepPlanToExecute.addFootstep(footstep.getDefinition().getSide(), solePose);
      }

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                    definition.getSwingDuration(),
                                                                                                                    definition.getTransferDuration());
      footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
      executionTimer.reset();

      nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                         definition.getSwingDuration(),
                                                                                         walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                         definition.getTransferDuration(),
                                                                                         walkingControllerParameters.getDefaultFinalTransferTime());

      for (RobotSide side : RobotSide.values)
         indexOfLastFoot.put(side, -1);
      for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
         indexOfLastFoot.put(footstepPlanToExecute.getFootstep(i).getRobotSide(), i);

      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

         int indexOfLastFootSide = indexOfLastFoot.get(side);
         if (indexOfLastFootSide >= 0)
         {
            goalFeetPoses.get(side).setIncludingFrame(state.getFootsteps().get(indexOfLastFootSide).getSoleFrame().getReferenceFrame().getParent(),
                                                      footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
            goalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }
         else
            goalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
      }
      startPositionDistanceToGoal =  0;
      startOrientationDistanceToGoal = 0;
      for (RobotSide side : RobotSide.values)
      {
         startPositionDistanceToGoal += syncedFeetPoses.get(side).getTranslation().differenceNorm(goalFeetPoses.get(side).getTranslation());
         startOrientationDistanceToGoal += syncedFeetPoses.get(side).getRotation().distance(goalFeetPoses.get(side).getRotation(), true);
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      boolean isComplete = true;
      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
         int indexOfLastFootSide = indexOfLastFoot.get(side);
         if (indexOfLastFootSide >= 0)
         {
            goalFeetPoses.get(side).setIncludingFrame(state.getFootsteps().get(indexOfLastFootSide).getSoleFrame().getReferenceFrame().getParent(),
                                                      footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
            goalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }
         else
            goalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));

         isComplete &= completionCalculator.get(side)
                                           .isComplete(goalFeetPoses.get(side),
                                                       syncedFeetPoses.get(side), POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
                                                       nominalExecutionDuration,
                                                       executionTimer,
                                                       BehaviorActionCompletionComponent.TRANSLATION,
                                                       BehaviorActionCompletionComponent.ORIENTATION);
      }
      int incompleteFootsteps = footstepTracker.getNumberOfIncompleteFootsteps();
      isComplete &= incompleteFootsteps == 0;

      isExecuting = !isComplete;

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(nominalExecutionDuration);
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
      executionStatusMessage.setNumberOfIncompleteFootsteps(incompleteFootsteps);
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getRotationError()
                                                                 + completionCalculator.get(RobotSide.RIGHT).getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getTranslationError()
                                                              + completionCalculator.get(RobotSide.RIGHT).getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }

   @Override
   public FootstepPlanActionState getState()
   {
      return state;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
