package us.ihmc.humanoidBehaviors.lookAndStep;

import java.util.UUID;
import java.util.function.Supplier;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.robotics.robotSide.SideDependentList;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;

   protected RobotWalkRequester robotWalkRequester;
   protected Runnable replanFootstepsOutput;

   protected FootstepPlan footstepPlan;
   protected RemoteSyncedRobotModel syncedRobot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected long previousStepMessageId = 0L;
   protected SideDependentList<PlannedFootstepReadOnly> lastCommandedFoosteps;

   public static class LookAndStepStepping extends LookAndStepSteppingTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<FootstepPlan> footstepPlanInput = new TypedInput<>();
      private BehaviorTaskSuppressor suppressor;
      private ControllerStatusTracker controllerStatusTracker;

      public void initialize(StatusLogger statusLogger,
                             RemoteSyncedRobotModel syncedRobot,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             UIPublisher uiPublisher,
                             RobotWalkRequester robotWalkRequester,
                             Runnable replanFootstepsOutput,
                             ControllerStatusTracker controllerStatusTracker,
                             Supplier<LookAndStepBehavior.State> behaviorStateReference,
                             SideDependentList<PlannedFootstepReadOnly> lastCommandedFoosteps)
      {
         this.controllerStatusTracker = controllerStatusTracker;
         this.lastCommandedFoosteps = lastCommandedFoosteps;
         this.statusLogger = statusLogger;
         this.syncedRobot = syncedRobot;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.uiPublisher = uiPublisher;
         this.robotWalkRequester = robotWalkRequester;
         this.replanFootstepsOutput = replanFootstepsOutput;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
         footstepPlanInput.addCallback(data -> executor.queueExecution(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Robot motion");
         suppressor.addCondition("Not in robot motion state", () -> !behaviorStateReference.get().equals(LookAndStepBehavior.State.STEPPING));
         suppressor.addCondition(() -> "Footstep plan not OK: numberOfSteps = " + (footstepPlan == null ? null : footstepPlan.getNumberOfSteps())
                                       + ". Planning again...",
                                 () -> !(footstepPlan != null && footstepPlan.getNumberOfSteps() > 0), replanFootstepsOutput::run);
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
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
      if (footstepPlan.getNumberOfSteps() > 0)
      {
         PlannedFootstep footstepToTake = footstepPlan.getFootstep(0);
         shortenedFootstepPlan.addFootstep(footstepToTake);
         lastCommandedFoosteps.put(footstepToTake.getRobotSide(), footstepToTake);
      }

      statusLogger.warn("Requesting walk");
      double swingTime = lookAndStepBehaviorParameters.getSwingTime();
      double transferTime = lookAndStepBehaviorParameters.getTransferTime();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime);
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
      double percentSwingToWait = lookAndStepBehaviorParameters.get(LookAndStepBehaviorParameters.percentSwingToWait);
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
