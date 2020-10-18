package us.ihmc.humanoidBehaviors.lookAndStep;

import java.util.UUID;
import java.util.function.Supplier;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepPlanEtcetera;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.LastCommandedFootsteps;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected FootstepPlanPostProcessHandler footstepPlanPostProcessor;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected SwingPlannerParametersReadOnly swingPlannerParameters;

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
                             FootstepPlannerParametersReadOnly footstepPlannerParameters,
                             SwingPlannerParametersReadOnly swingPlannerParameters,
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
         this.footstepPlannerParameters = footstepPlannerParameters;
         this.swingPlannerParameters = swingPlannerParameters;
         this.uiPublisher = uiPublisher;
         this.footstepPlanPostProcessor = footstepPlanPostProcessor;
         this.robotWalkRequester = robotWalkRequester;
         this.replanFootstepsOutput = replanFootstepsOutput;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
         footstepPlanEtcInput.addCallback(data -> executor.submitTask(this::evaluateAndRun));

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

      // TODO: Clean this up.
      // Extract swing time calculation from the proportion swing planner
      PlannedFootstep endStep = shortenedFootstepPlan.getFootstep(0);
      Pose3DReadOnly startStep = footstepPlanEtc.getStartFootPoses().get(endStep.getRobotSide());
      double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();
      double maxStepZ = footstepPlannerParameters.getMaxStepZ();
      double maximumStepDistance = EuclidCoreTools.norm(footstepPlannerParameters.getMaximumStepReach(), maxStepZ);
      double stepDistance = startStep.getPosition().distance(endStep.getFootstepPose().getPosition());
      double alpha = MathTools.clamp((stepDistance - idealStepLength) / (maximumStepDistance - idealStepLength), 0.0, 1.0);
      double swingDuration = swingPlannerParameters.getMinimumSwingTime()
            + alpha * (swingPlannerParameters.getMaximumSwingTime() - swingPlannerParameters.getMinimumSwingTime());
      if (endStep.getSwingDuration() < swingDuration)
      {
         statusLogger.info("Increasing swing duration to {} s", swingDuration);
         endStep.setSwingDuration(swingDuration);
      }
      swingDuration = endStep.getSwingDuration();

      lastCommandedFoosteps.put(footstepToTake.getRobotSide(), footstepToTake);
      uiPublisher.publishToUI(LastCommandedFootsteps, MinimalFootstep.reduceFootstepsForUIMessager(lastCommandedFoosteps));

      statusLogger.warn("Requesting walk");
      double transferTime = lookAndStepBehaviorParameters.getTransferTime();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingDuration,
                                                                                                                    transferTime);

      ExecutionMode executionMode = previousStepMessageId == 0L ? ExecutionMode.OVERRIDE : ExecutionMode.QUEUE;
      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      long messageId = UUID.randomUUID().getLeastSignificantBits();
      footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
      previousStepMessageId = messageId;
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotWalkRequester.requestWalk(footstepDataListMessage);

      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
      sleepForPartOfSwingThread(swingDuration);
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
