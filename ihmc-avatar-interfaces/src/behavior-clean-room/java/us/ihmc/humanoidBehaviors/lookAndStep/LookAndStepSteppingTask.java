package us.ihmc.humanoidBehaviors.lookAndStep;

import java.util.UUID;
import java.util.function.Supplier;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.footstepPlanning.*;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepPlanEtcetera;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.FootstepPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.LastCommandedFootsteps;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected FootstepPlanPostProcessHandler footstepPlanPostProcessor;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;

   protected RobotWalkRequester robotWalkRequester;
   protected Runnable replanFootstepsOutput;

   protected FootstepPlanEtcetera footstepPlanEtc;
   protected RemoteSyncedRobotModel syncedRobot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected long previousStepMessageId = 0L;
   protected SideDependentList<PlannedFootstepReadOnly> lastCommandedFoosteps;

   public static class LookAndStepStepping extends LookAndStepSteppingTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<FootstepPlanEtcetera> footstepPlanEtcInput = new TypedInput<>();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(StatusLogger statusLogger,
                             RemoteSyncedRobotModel syncedRobot,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             UIPublisher uiPublisher,
                             FootstepPlanPostProcessHandler footstepPlanPostProcessor,
                             RobotWalkRequester robotWalkRequester,
                             Runnable replanFootstepsOutput,
                             ControllerStatusTracker controllerStatusTracker,
                             Supplier<LookAndStepBehavior.State> behaviorStateReference,
                             SideDependentList<PlannedFootstepReadOnly> lastCommandedFoosteps)
      {
         this.lastCommandedFoosteps = lastCommandedFoosteps;
         this.statusLogger = statusLogger;
         this.syncedRobot = syncedRobot;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.uiPublisher = uiPublisher;
         this.footstepPlanPostProcessor = footstepPlanPostProcessor;
         this.robotWalkRequester = robotWalkRequester;
         this.replanFootstepsOutput = replanFootstepsOutput;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
         footstepPlanEtcInput.addCallback(data -> executor.queueExecution(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Robot motion");
         suppressor.addCondition("Not in robot motion state", () -> !behaviorStateReference.get().equals(LookAndStepBehavior.State.STEPPING));
         suppressor.addCondition(() -> "Footstep plan not OK: numberOfSteps = " + (footstepPlanEtc == null ? null : footstepPlanEtc.getNumberOfSteps())
                                       + ". Planning again...",
                                 () -> !(footstepPlanEtc != null && footstepPlanEtc.getNumberOfSteps() > 0), replanFootstepsOutput);
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
      }

      public void reset()
      {
         executor.interruptAndReset();
         previousStepMessageId = 0L;
      }

      public void acceptFootstepPlan(FootstepPlanEtcetera footstepPlanEtc)
      {
         footstepPlanEtcInput.set(footstepPlanEtc);
      }

      private void evaluateAndRun()
      {
         footstepPlanEtc = footstepPlanEtcInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepBehaviorParameters.getRobotConfigurationDataExpiration());

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   protected void performTask()
   {
      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      PlannedFootstep footstepToTake = footstepPlanEtc.getFootstep(0);
      shortenedFootstepPlan.addFootstep(footstepToTake);

      footstepPlanPostProcessor.performPostProcessing(footstepPlanEtc.getPlanarRegions(),
                                                      shortenedFootstepPlan,
                                                      footstepPlanEtc.getStartFootPoses(),
                                                      footstepPlanEtc.getStartFootholds(),
                                                      footstepPlanEtc.getSwingPlannerType(),
                                                      true,
                                                      true);

      lastCommandedFoosteps.put(footstepToTake.getRobotSide(), footstepToTake);
      uiPublisher.publishToUI(LastCommandedFootsteps, MinimalFootstep.reduceFootstepsForUIMessager(lastCommandedFoosteps));

      statusLogger.warn("Requesting walk");
      double swingTime = lookAndStepBehaviorParameters.getSwingTime();
      double transferTime = lookAndStepBehaviorParameters.getTransferTime();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime);
      for (FootstepDataMessage footstepDataMessage : footstepDataListMessage.getFootstepDataList())
      {
         if (footstepDataMessage.getSwingDuration() > 0)
         {
            statusLogger.warn("Overriding planned swing duration {} -> {}", footstepDataMessage.getSwingDuration(), swingTime);
            footstepDataMessage.setSwingDuration(swingTime);
         }
         if (footstepDataMessage.getTransferDuration() > 0)
         {
            statusLogger.warn("Overriding planned transfer duration {} -> {}", footstepDataMessage.getTransferDuration(), transferTime);
            footstepDataMessage.setTransferDuration(transferTime);
         }
      }

      ExecutionMode executionMode = previousStepMessageId == 0L ? ExecutionMode.OVERRIDE : ExecutionMode.QUEUE;
      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      long messageId = UUID.randomUUID().getLeastSignificantBits();
      footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
      previousStepMessageId = messageId;
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotWalkRequester.requestWalk(footstepDataListMessage);

      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
      sleepForPartOfSwingThread(swingTime);
   }

   private void sleepForPartOfSwingThread(double swingTime)
   {
      double percentSwingToWait = lookAndStepBehaviorParameters.getPercentSwingToWait();
      double waitTime = swingTime * percentSwingToWait;
      statusLogger.info("Waiting {} s for {} % of swing...", waitTime, percentSwingToWait);
      ThreadTools.sleepSeconds(waitTime);
      statusLogger.info("{} % of swing complete!", percentSwingToWait);
      replanFootstepsOutput.run();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      statusLogger.debug("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      statusLogger.debug("Robot walk complete.");
   }
}
