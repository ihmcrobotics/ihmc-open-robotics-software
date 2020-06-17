package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.FootstepPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.StartAndGoalFootPosesForUI;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.humanoidBehaviors.tools.BehaviorBuilderPattern;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidBehaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.robotics.robotSide.RobotSide;

class LookAndStepRobotMotionTask implements BehaviorBuilderPattern
{
   protected final StatusLogger statusLogger;

   private final Field<LookAndStepBehaviorParametersReadOnly> lookAndStepBehaviorParameters = required();
   private final Field<BiConsumer<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseConsumer = required();
   private final Field<Function<RobotSide, FramePose3DReadOnly>> lastSteppedSolePoseSupplier = required();
   private final Field<UIPublisher> uiPublisher = required();
   private final Field<RobotWalkRequester> robotWalkRequester = required();
   private final Field<Runnable> replanFootstepsOutput = required();
   protected final Field<Consumer<LookAndStepBehavior.State>> behaviorStateUpdater = required();

   private FootstepPlan footstepPlan;
   private HumanoidRobotState robotState;
   private LookAndStepBehavior.State behaviorState;

   LookAndStepRobotMotionTask(StatusLogger statusLogger)
   {
      this.statusLogger = statusLogger;
   }

   protected void update(FootstepPlan footstepPlan,
                         HumanoidRobotState robotState,
                         LookAndStepBehavior.State behaviorState)
   {
      this.footstepPlan = footstepPlan;
      this.robotState = robotState;
      this.behaviorState = behaviorState;
   }

   private boolean evaluateEntry()
   {
      boolean proceed = true;

      if (!behaviorState.equals(LookAndStepBehavior.State.SWINGING))
      {
         statusLogger.debug("Footstep planning supressed: Not in footstep planning state");
         proceed = false;
      }
      else if (!isFootstepPlanOK())
      {
         statusLogger.debug("Robot walking supressed: Footstep plan not OK: numberOfSteps = {}. Planning again...",
                       footstepPlan == null ? null : footstepPlan.getNumberOfSteps());
         behaviorStateUpdater.get().accept(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
         replanFootstepsOutput.get().run();
         proceed = false;
      }

      return proceed;
   }

   private boolean isFootstepPlanOK()
   {
      return footstepPlan != null && footstepPlan.getNumberOfSteps() > 0; // TODO: Shouldn't we prevent ever getting here?
   }

   private void performTask()
   {
      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      if (footstepPlan.getNumberOfSteps() > 0)
      {
         PlannedFootstep footstepToTake = footstepPlan.getFootstep(0);
         shortenedFootstepPlan.addFootstep(footstepToTake);
         lastSteppedSolePoseConsumer.get().accept(footstepToTake.getRobotSide(), new FramePose3D(footstepToTake.getFootstepPose()));
      }
      ArrayList<FootstepForUI> startFootPosesForUI = new ArrayList<>();
      startFootPosesForUI.add(new FootstepForUI(RobotSide.LEFT, new Pose3D(lastSteppedSolePoseSupplier.get().apply(RobotSide.LEFT)), "Left Start"));
      startFootPosesForUI.add(new FootstepForUI(RobotSide.RIGHT, new Pose3D(lastSteppedSolePoseSupplier.get().apply(RobotSide.RIGHT)), "Right Start"));
      uiPublisher.get().publishToUI(StartAndGoalFootPosesForUI, startFootPosesForUI); // TODO: Should specify topic here?

      statusLogger.info("Requesting walk");
      double swingTime = lookAndStepBehaviorParameters.get().getSwingTime();
      double transferTime = lookAndStepBehaviorParameters.get().getTransferTime();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime);
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotWalkRequester.get().requestWalk(footstepDataListMessage);

      uiPublisher.get()
                 .publishToUI(FootstepPlanForUI,
                              FootstepForUI.reduceFootstepPlanForUIMessager(FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage),
                                                                            "Stepping"));

      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
      sleepForPartOfSwingThread(swingTime);
   }

   private void sleepForPartOfSwingThread(double swingTime)
   {
      double percentSwingToWait = lookAndStepBehaviorParameters.get().get(LookAndStepBehaviorParameters.percentSwingToWait);
      double waitTime = swingTime * percentSwingToWait;
      statusLogger.info("Waiting {} for {} % of swing...", waitTime, percentSwingToWait);
      ThreadTools.sleepSeconds(waitTime);
      statusLogger.info("{} % of swing complete!", percentSwingToWait);
      behaviorStateUpdater.get().accept(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
      replanFootstepsOutput.get();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      statusLogger.info("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      statusLogger.info("Robot walk complete.");
   }

   public void run()
   {
      validateAll();

      if (evaluateEntry())
      {
         performTask();
      }

      invalidateChanging();
   }

   public void setLookAndStepBehaviorParameters(LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters)
   {
      this.lookAndStepBehaviorParameters.set(lookAndStepBehaviorParameters);
   }

   public void setLastSteppedSolePoseConsumer(BiConsumer<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseConsumer)
   {
      this.lastSteppedSolePoseConsumer.set(lastSteppedSolePoseConsumer);
   }

   public void setLastSteppedSolePoseSupplier(Function<RobotSide, FramePose3DReadOnly> lastSteppedSolePoseSupplier)
   {
      this.lastSteppedSolePoseSupplier.set(lastSteppedSolePoseSupplier);
   }

   public void setUiPublisher(UIPublisher uiPublisher)
   {
      this.uiPublisher.set(uiPublisher);
   }

   public void setRobotWalkRequester(RobotWalkRequester robotWalkRequester)
   {
      this.robotWalkRequester.set(robotWalkRequester);
   }

   public void setReplanFootstepsOutput(Runnable replanFootstepsOutput)
   {
      this.replanFootstepsOutput.set(replanFootstepsOutput);
   }

   public void setBehaviorStateUpdater(Consumer<LookAndStepBehavior.State> behaviorStateUpdater)
   {
      this.behaviorStateUpdater.set(behaviorStateUpdater);
   }
}
