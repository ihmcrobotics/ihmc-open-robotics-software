package us.ihmc.humanoidBehaviors.lookAndStep;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.FootstepPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.StartAndGoalFootPosesForUI;

import java.util.ArrayList;
import java.util.UUID;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.util.TimerSnapshotWithExpiration;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;

   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses;
   protected UIPublisher uiPublisher;
   protected RobotWalkRequester robotWalkRequester;
   protected Runnable replanFootstepsOutput;
   protected BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;

   protected FootstepPlan footstepPlan;
   protected RemoteSyncedRobotModel syncedRobot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected long previousStepMessageId = 0L;

   public static class LookAndStepStepping extends LookAndStepSteppingTask
   {
      private final TypedInput<FootstepPlan> footstepPlanInput = new TypedInput<>();
      private BehaviorTaskSuppressor suppressor;
      private ControllerStatusTracker controllerStatusTracker;

      public void initialize(StatusLogger statusLogger,
                             RemoteSyncedRobotModel syncedRobot,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             SideDependentList<FramePose3DReadOnly> lastSteppedSolePoses,
                             UIPublisher uiPublisher,
                             RobotWalkRequester robotWalkRequester,
                             Runnable replanFootstepsOutput,
                             ControllerStatusTracker controllerStatusTracker,
                             BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference)
      {
         this.controllerStatusTracker = controllerStatusTracker;
         this.statusLogger = statusLogger;
         this.syncedRobot = syncedRobot;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.lastSteppedSolePoses = lastSteppedSolePoses;
         this.uiPublisher = uiPublisher;
         this.robotWalkRequester = robotWalkRequester;
         this.replanFootstepsOutput = replanFootstepsOutput;
         this.behaviorStateReference = behaviorStateReference;

         SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
         footstepPlanInput.addCallback(data -> executor.execute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Robot motion");
         suppressor.addCondition("Not in robot motion state", () -> !behaviorStateReference.get().equals(LookAndStepBehavior.State.STEPPING));
         suppressor.addCondition(() -> "Footstep plan not OK: numberOfSteps = " + (footstepPlan == null ? null : footstepPlan.getNumberOfSteps())
                                       + ". Planning again...",
                                 () -> !(footstepPlan != null && footstepPlan.getNumberOfSteps() > 0),
                                 () ->
                                 {
                                    behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
                                    replanFootstepsOutput.run();
                                 });
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition("Robot not in walking state", () -> !controllerStatusTracker.isInWalkingState());
      }

      public void acceptFootstepPlan(FootstepPlan footstepPlan)
      {
         footstepPlanInput.set(footstepPlan);
      }

      private void evaluateAndRun()
      {
         footstepPlan = footstepPlanInput.get();
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
         lastSteppedSolePoses.put(footstepToTake.getRobotSide(), new FramePose3D(footstepToTake.getFootstepPose()));
      }
      ArrayList<FootstepForUI> startFootPosesForUI = new ArrayList<>();
      startFootPosesForUI.add(new FootstepForUI(RobotSide.LEFT, new Pose3D(lastSteppedSolePoses.get(RobotSide.LEFT)), "Left Start"));
      startFootPosesForUI.add(new FootstepForUI(RobotSide.RIGHT, new Pose3D(lastSteppedSolePoses.get(RobotSide.RIGHT)), "Right Start"));
      uiPublisher.publishToUI(StartAndGoalFootPosesForUI, startFootPosesForUI); // TODO: Should specify topic here?

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

      uiPublisher.publishToUI(FootstepPlanForUI,
                              FootstepForUI.reduceFootstepPlanForUIMessager(FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage),
                                                                            "Stepping"));

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
      behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
      replanFootstepsOutput.run();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      statusLogger.debug("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      statusLogger.debug("Robot walk complete.");
   }
}
