package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
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

public class FootstepPlanAction extends FootstepPlanActionData implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
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

   public FootstepPlanAction(ROS2ControllerHelper ros2ControllerHelper,
                             ROS2SyncedRobotModel syncedRobot,
                             WalkingFootstepTracker footstepTracker,
                             ReferenceFrameLibrary referenceFrameLibrary,
                             WalkingControllerParameters walkingControllerParameters)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.walkingControllerParameters = walkingControllerParameters;
      setReferenceFrameLibrary(referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      update();

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      footstepPlanToExecute.clear();
      for (FootstepActionData footstep : getFootsteps())
      {
         solePose.setIncludingFrame(getPlanFrame(), footstep.getSolePose());
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         footstepPlanToExecute.addFootstep(footstep.getSide(), solePose);
      }

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                    getSwingDuration(),
                                                                                                                    getTransferDuration());
      footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
      executionTimer.reset();

      nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                         getSwingDuration(),
                                                                                         walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                         getTransferDuration(),
                                                                                         walkingControllerParameters.getDefaultFinalTransferTime());

      for (RobotSide side : RobotSide.values)
         indexOfLastFoot.put(side, -1);
      for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
         indexOfLastFoot.put(footstepPlanToExecute.getFootstep(i).getRobotSide(), i);

      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

         if (indexOfLastFoot.get(side) >= 0)
         {
            goalFeetPoses.get(side).setIncludingFrame(getPlanFrame(), footstepPlanToExecute.getFootstep(indexOfLastFoot.get(side)).getFootstepPose());
            goalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }
         else
            goalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      boolean isComplete = true;
      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

         isComplete &= completionCalculator.get(side)
                                           .isComplete(goalFeetPoses.get(side),
                                                       syncedFeetPoses.get(side), POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
                                                       nominalExecutionDuration,
                                                       executionTimer);
      }
      int incompleteFootsteps = footstepTracker.getNumberOfIncompleteFootsteps();
      isComplete &= incompleteFootsteps == 0;

      isExecuting = !isComplete;

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(nominalExecutionDuration);
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
      executionStatusMessage.setNumberOfIncompleteFootsteps(incompleteFootsteps);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getRotationError()
                                                                 + completionCalculator.get(RobotSide.RIGHT).getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getTranslationError()
                                                              + completionCalculator.get(RobotSide.RIGHT).getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, this.executionStatusMessage);
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }
}
