package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class RecoveringSwingState extends PushRecoveryState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int additionalFootstepsToConsider;
   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private final Footstep[] footsteps;
   private final FootstepTiming[] footstepTimings;

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final Runnable stepStartedListener;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final PushRecoveryControllerParameters pushRecoveryParameters;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   protected final YoDouble minimumReactionTime = new YoDouble("minimumReactionTime", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final FullHumanoidRobotModel fullRobotModel;

   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;
   private final PushRecoveryBalanceManager balanceManager;

   public RecoveringSwingState(PushRecoveryStateEnum stateEnum,
                               WalkingMessageHandler walkingMessageHandler,
                               HighLevelHumanoidControllerToolbox controllerToolbox,
                               PushRecoveryControlManagerFactory managerFactory,
                               MultiStepPushRecoveryControlModule pushRecoveryControlModule,
                               PushRecoveryControllerParameters pushRecoveryParameters,
                               Runnable stepStartedListener,
                               WalkingFailureDetectionControlModule failureDetectionControlModule,
                               YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);
      minimumReactionTime.set(0.15);

      this.supportSide = stateEnum.getSupportSide();
      this.pushRecoveryControlModule = pushRecoveryControlModule;
      this.pushRecoveryParameters = pushRecoveryParameters;
      this.stepStartedListener = stepStartedListener;
      swingSide = supportSide.getOppositeSide();

      this.walkingMessageHandler = walkingMessageHandler;
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      this.balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      additionalFootstepsToConsider = balanceManager.getMaxNumberOfStepsToConsider();
      footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
      footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   @Override
   public RobotSide getSupportSide()
   {
      return supportSide;
   }


   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();

      walkingMessageHandler.clearFlamingoCommands();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));

      return hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGroundFiltered();
   }

   @Override
   public void onEntry()
   {
      stepStartedListener.run();

      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();

      comHeightManager.setSupportLeg(swingSide.getOppositeSide());

      nextFootstep.set(pushRecoveryControlModule.getRecoveryStep(0));
      footstepTiming.set(pushRecoveryControlModule.getRecoveryStepTiming(0));

      nextFootstep.setTrajectoryType(TrajectoryType.DEFAULT);
      swingTime = footstepTiming.getSwingTime();

      failureDetectionControlModule.setNextFootstep(nextFootstep);

      /** 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the contact points are up to date */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

      double finalTransferTime = pushRecoveryParameters.getFinalTransferDurationForRecovery();

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, pushRecoveryControlModule.getNumberOfRecoverySteps() - 1);
      for (int i = 1; i < stepsToAdd; i++)
      {
         footsteps[i].set(pushRecoveryControlModule.getRecoveryStep(i));
         footstepTimings[i].set(pushRecoveryControlModule.getRecoveryStepTiming(i));

         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
      }

      balanceManager.initializeICPPlanForSingleSupport();

      updateHeightManager();

      feetManager.requestSwing(swingSide, nextFootstep, swingTime, null, null);

      pelvisOrientationManager.initializeSwing();

      actualFootPoseInWorld.setFromReferenceFrame(fullRobotModel.getSoleFrame(swingSide));

      walkingMessageHandler.reportFootstepStarted(swingSide, nextFootstep.getFootstepPose(), actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
   }

   @Override
   public void onExit(double timeInState)
   {
      actualFootPoseInWorld.setFromReferenceFrame(fullRobotModel.getSoleFrame(swingSide));

      walkingMessageHandler.reportFootstepCompleted(swingSide, nextFootstep.getFootstepPose(), actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      feetManager.touchDown(swingSide, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }


   private void updateFootstepParameters()
   {
      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated
   }

   private void updateHeightManager()
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep,
                                                                                                                                                swingSide);
      transferToAndNextFootstepsData.setComAtEndOfState(balanceManager.getFinalDesiredCoMPosition());
      double extraToeOffHeight = 0.0;
      if (feetManager.canDoSingleSupportToeOff(nextFootstep.getFootstepPose(), swingSide) && feetManager.isSteppingUp())
         extraToeOffHeight = feetManager.getExtraCoMMaxHeightWithToes();
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   private boolean hasMinimumTimePassed(double timeInState)
   {
      return timeInState > minimumReactionTime.getDoubleValue();
   }
}