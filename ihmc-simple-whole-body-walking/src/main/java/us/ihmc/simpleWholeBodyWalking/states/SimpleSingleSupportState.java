package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.simpleWholeBodyWalking.SimplePelvisOrientationManager;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleSingleSupportState extends SimpleWalkingState
{
   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   protected final YoDouble minimumSwingFraction = new YoDouble("minimumSwingFraction", registry);

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final SimpleBalanceManager balanceManager;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private double swingTime;

   private static final int additionalFootstepsToConsider = 2;
   private final Footstep[] footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);
   private final FramePose3D desiredFootPoseInWorld = new FramePose3D(worldFrame);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;

   private final FramePoint3D desiredCoM = new FramePoint3D();

   private final YoDouble remainingSwingTimeAccordingToPlan = new YoDouble("remainingSwingTimeAccordingToPlan", registry);
   private final YoDouble estimatedRemainingSwingTimeUnderDisturbance = new YoDouble("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final YoDouble icpErrorThresholdToSpeedUpSwing = new YoDouble("icpErrorThresholdToSpeedUpSwing", registry);

   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public SimpleSingleSupportState(SimpleWalkingStateEnum stateEnum,
                                   WalkingMessageHandler walkingMessageHandler,
                                   HighLevelHumanoidControllerToolbox controllerToolbox,
                                   SimpleControlManagerFactory managerFactory,
                                   WalkingControllerParameters walkingControllerParameters,
                                   WalkingFailureDetectionControlModule failureDetectionControlModule,
                                   YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.supportSide = stateEnum.getSupportSide();
      swingSide = supportSide.getOppositeSide();

      minimumSwingFraction.set(0.5);

      this.walkingMessageHandler = walkingMessageHandler;
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      balanceManager = managerFactory.getOrCreateBalanceManager();

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());

      setYoVariablesToNaN();
   }

   @Override
   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   @Override
   public void doAction(double timeInState)
   {

      boolean requestSwingSpeedUp = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

      boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateFootstepFromICPOptimization(nextFootstep);

      if (footstepIsBeingAdjusted)
      {
         requestSwingSpeedUp = true;
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
         updateFootstepParameters();

         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

      }

      // if the footstep was adjusted, shift the CoM plan, if there is one.
      walkingMessageHandler.setPlanOffsetFromAdjustment(balanceManager.getEffectiveICPAdjustment());

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }

      walkingMessageHandler.clearFootTrajectory();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));

      if (hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround())
         return true;

      return balanceManager.isICPPlanDone();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      swingTime = walkingMessageHandler.getNextSwingTime();
      walkingMessageHandler.poll(nextFootstep, footstepTiming);

      /** 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the contact points are up to date */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      boolean isLastStep = stepsToAdd == 0;
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport(finalTransferTime);

      feetManager.requestSwing(swingSide, nextFootstep, swingTime);

      if (feetManager.adjustHeightIfNeeded(nextFootstep))
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);
      }

      if (isLastStep)
      {
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, finalTransferTime, 0.0);
      }
      else
      {
         FootstepTiming nextTiming = footstepTimings[0];
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, nextTiming.getTransferTime(), nextTiming.getSwingTime());
      }

      nextFootstep.getPose(desiredFootPoseInWorld);
      desiredFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
   }

   @Override
   public void onExit(double timeInState)
   {
      feetManager.setFlatFootContactState(swingSide);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      walkingMessageHandler.reportFootstepCompleted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime, nextFootstep.getSequenceID());
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(nextFootstep.getRobotSide());
      tempOrientation.setIncludingFrame(nextFootstep.getFootstepPose().getOrientation());
      tempOrientation.changeFrame(soleZUpFrame);

      setYoVariablesToNaN();
   }

   private void setYoVariablesToNaN()
   {
      estimatedRemainingSwingTimeUnderDisturbance.setToNaN();
      remainingSwingTimeAccordingToPlan.setToNaN();
   }

   /**
    * Request the swing trajectory to speed up using
    * {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(FramePoint2DReadOnly)}
    * It is clamped w.r.t. to
    * {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    *
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpIfNeeded()
   {
      remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());

      double remainingTime = 0;
      estimatedRemainingSwingTimeUnderDisturbance.set(remainingTime);

      if (remainingTime > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / remainingTime;
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan.getDoubleValue();
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

  

   protected boolean hasMinimumTimePassed(double timeInState)
   {
      double minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }
}