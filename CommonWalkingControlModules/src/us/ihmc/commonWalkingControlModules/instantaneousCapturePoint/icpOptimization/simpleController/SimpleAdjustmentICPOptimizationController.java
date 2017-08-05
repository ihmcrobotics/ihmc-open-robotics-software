package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.*;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleAdjustmentICPOptimizationController extends AbstractSimpleICPOptimizationController
{
   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   public SimpleAdjustmentICPOptimizationController(ICPOptimizationParameters icpOptimizationParameters, WalkingControllerParameters walkingControllerParameters,
                                                    BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                                    double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(icpOptimizationParameters, bipedSupportPolygons, contactableFeet, controlDT, yoGraphicsListRegistry);

      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());
      else
         swingSpeedUpEnabled.set(false);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInDoubleSupport.set(true);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);

      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());

      speedUpTime.set(0.0);
   }

   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.transferToSide.set(transferToSide);
      isInDoubleSupport.set(true);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);
   }

   @Override
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInDoubleSupport.set(false);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForSingleSupport(supportSide, solver);
   }

   @Override
   protected void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      super.initializeOnContactChange(initialTime);
   }

   @Override
   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP, double omega0)
   {
      controllerTimer.startMeasurement();

      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      this.currentICP.set(currentICP);
      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);

      this.icpError.set(currentICP);
      this.icpError.sub(desiredICP);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega0, perfectCMP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem();

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(numberOfFootstepsToConsider, omega0);

      qpSolverTimer.startMeasurement();
      NoConvergenceException noConvergenceException = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(numberOfFootstepsToConsider, noConvergenceException);

      modifyAngularMomentumWeightUsingIntegral();

      controllerTimer.stopMeasurement();
   }

   @Override
   public double getOptimizedTimeRemaining()
   {
      throw new RuntimeException("This is not implemented in this solver.");
   }

   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      if (swingSpeedUpEnabled.getBooleanValue() && remainingTimeForSwing < timeRemainingInState.getDoubleValue())
      {
         double speedUpTime = timeRemainingInState.getDoubleValue() - remainingTimeForSwing;
         this.speedUpTime.add(speedUpTime);
      }
   }

   @Override
   protected void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
   }
}
