package us.ihmc.behaviors.lookAndStep;

import java.util.UUID;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.footstepPlanning.*;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.behaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected BehaviorHelper helper;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;

   protected RobotWalkRequester robotWalkRequester;
   protected Runnable doneWaitingForSwingOutput;

   protected FootstepPlan footstepPlan;
   protected ROS2SyncedRobotModel syncedRobot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected long previousStepMessageId = 0L;
   protected LookAndStepImminentStanceTracker imminentStanceTracker;
   protected ControllerStatusTracker controllerStatusTracker;
   protected double stepDuration;
   private final Timer timerSincePlanWasSent = new Timer();

   protected final TypedInput<RobotConfigurationData> robotConfigurationData = new TypedInput<>();

   public static class LookAndStepStepping extends LookAndStepSteppingTask
   {
      private ResettableExceptionHandlingExecutorService executor;
      private final TypedInput<FootstepPlan> footstepPlanInput = new TypedInput<>();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         controllerStatusTracker = lookAndStep.controllerStatusTracker;
         imminentStanceTracker = lookAndStep.imminentStanceTracker;
         statusLogger = lookAndStep.statusLogger;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         helper = lookAndStep.helper;
         uiPublisher = lookAndStep.helper::publish;
         robotWalkRequester = lookAndStep.robotInterface::requestWalk;
         doneWaitingForSwingOutput = () ->
         {
            if (!lookAndStep.isBeingReset.get())
            {
               lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
               lookAndStep.bodyPathLocalization.acceptSwingSleepComplete();
            }
         };

         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
         footstepPlanInput.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Stepping task");
         suppressor.addCondition("Not in robot motion state", () -> !lookAndStep.behaviorStateReference.get().equals(LookAndStepBehavior.State.STEPPING));
         suppressor.addCondition(() -> "Footstep plan not OK: numberOfSteps = " + (footstepPlan == null ? null : footstepPlan.getNumberOfSteps())
                                       + ". Planning again...", () -> !(footstepPlan != null && footstepPlan.getNumberOfSteps() > 0), doneWaitingForSwingOutput);
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition("Robot not in walking state", () -> !lookAndStep.controllerStatusTracker.isInWalkingState());

         lookAndStepParameters = syncedRobot.getRobotModel().getLookAndStepParameters();
      }

      public void reset()
      {
         executor.interruptAndReset();
         previousStepMessageId = 0L;
      }

      public void acceptFootstepPlan(FootstepPlan footstepPlan)
      {
         footstepPlanInput.set(footstepPlan);
      }

      private void evaluateAndRun()
      {
         footstepPlan = footstepPlanInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepParameters.getRobotConfigurationDataExpiration());

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   private final Stopwatch stepDurationStopwatch = new Stopwatch();

   protected void performTask()
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);
      FootstepDataMessageConverter.appendPlanToMessage(footstepPlan, footstepDataListMessage);

      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         footstepDataListMessage.getFootstepDataList().get(i).setShouldCheckForReachability(true);
      }

      // TODO: Add combo to look and step UI to chose which steps to visualize
      helper.publish(LAST_COMMANDED_FOOTSTEPS, MinimalFootstep.convertToMinimalFootstepListMessage(footstepDataListMessage, "Look and Step Last Commanded"));

      ExecutionMode executionMode = ExecutionMode.OVERRIDE; // Always override, which we can do now
      imminentStanceTracker.addCommandedFootsteps(footstepPlan);

      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      long messageId = UUID.randomUUID().getLeastSignificantBits();
      footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
      previousStepMessageId = messageId;
      statusLogger.warn("Requesting walk {}ing {} step plan starting with {} foot.",
                        executionMode.name(),
                        footstepPlan.getNumberOfSteps(),
                        footstepPlan.getFootstep(0).getRobotSide().name());
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotWalkRequester.requestWalk(footstepDataListMessage);
      timerSincePlanWasSent.reset();

      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
      // TODO: Where do we wait for transfer? We should consider possible exploitations of transfer time to do useful stuff,
      // TODO: but also make it explict where it is happening, wait for it, know when it stops and starts relative to this code
      waitForPartOfSwing(lookAndStepParameters.getSwingDuration());
   }

   private void waitForPartOfSwing(double swingDuration)
   {
      double estimatedRobotTimeWhenPlanWasSent = getEstimatedRobotTime();
      double percentSwingToWait = lookAndStepParameters.getPercentSwingToWait();
      double waitDuration = swingDuration * percentSwingToWait;
      double maxDurationToWait = 10.0;
      double robotTimeToStopWaitingRegardless = estimatedRobotTimeWhenPlanWasSent + maxDurationToWait;
      statusLogger.info("Waiting up to {} s for commanded step to start...", maxDurationToWait);
      stepDurationStopwatch.reset();

      boolean stepStartTimeRecorded = false;
      double robotTimeInWhichStepStarted = Double.NaN;
      double robotTimeToStopWaiting = Double.NaN;
      while (true)
      {
         double moreRobustRobotTime = getMoreRobustRobotTime(estimatedRobotTimeWhenPlanWasSent);
         boolean stepHasStarted = imminentStanceTracker.getFirstCommandedStepHasStarted();
         boolean haveWaitedMaxDuration = moreRobustRobotTime >= robotTimeToStopWaitingRegardless;
         boolean robotIsNotWalkingAnymoreForSomeReason = !controllerStatusTracker.isWalking();
         boolean stepCompletedEarly = imminentStanceTracker.getFirstCommandedStepHasCompleted();

         // Part 1: Wait for the step to start with a timeout
         if (haveWaitedMaxDuration)
         {
            statusLogger.info("Waited max duration of {} s. Done waiting.", maxDurationToWait);
            break;
         }

         // Part 2: The commanded step has started
         if (stepHasStarted)
         {
            if (!stepStartTimeRecorded)
            {
               stepStartTimeRecorded = true;
               robotTimeInWhichStepStarted = getMoreRobustRobotTime(estimatedRobotTimeWhenPlanWasSent);
               robotTimeToStopWaiting = moreRobustRobotTime + waitDuration;
               statusLogger.info("Waiting {} s for {} % of swing...", waitDuration, percentSwingToWait * 100.0);
            }

            if (robotIsNotWalkingAnymoreForSomeReason)
            {
               statusLogger.info("Robot not walking anymore {} s after step started for some reason. Done waiting.",
                                 moreRobustRobotTime - robotTimeInWhichStepStarted);
               break;
            }
            else if (stepCompletedEarly)
            {
               statusLogger.info("Step completed {} s early. Done waiting.",
                                 FormattingTools.getFormattedDecimal3D(robotTimeToStopWaiting - moreRobustRobotTime));
               break;
            }
            else if (moreRobustRobotTime >= robotTimeToStopWaiting)
            {
               double durationSinceSwingStarted = moreRobustRobotTime - robotTimeInWhichStepStarted;
               double durationWaited = moreRobustRobotTime - estimatedRobotTimeWhenPlanWasSent;
               statusLogger.info("{} % of swing complete! Done waiting. We waited for {} s since step started. ({} s total)",
                                 FormattingTools.getFormattedDecimal3D(durationSinceSwingStarted / swingDuration * 100.0),
                                 FormattingTools.getFormattedDecimal3D(durationSinceSwingStarted),
                                 FormattingTools.getFormattedDecimal3D(durationWaited));
               break;
            }
         }

         ThreadTools.sleepSeconds(0.01); // Prevent free spinning
      }

      stepDuration = stepDurationStopwatch.lap();

      doneWaitingForSwingOutput.run();
   }

   private double getMoreRobustRobotTime(double estimatedRobotTimeWhenPlanWasSent)
   {
      return Math.max(getEstimatedRobotTime(), estimatedRobotTimeWhenPlanWasSent + timerSincePlanWasSent.getElapsedTime());
   }

   private double getEstimatedRobotTime()
   {
      return Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()) + syncedRobot.getDataReceptionTimerSnapshot().getTimePassedSinceReset();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      statusLogger.debug("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      statusLogger.debug("Robot walk complete.");
   }
}
