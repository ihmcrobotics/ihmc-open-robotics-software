package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPAdjustmentOptimizationController extends AbstractICPOptimizationController
{
   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   public ICPAdjustmentOptimizationController(ICPPlannerParameters icpPlannerParameters, WalkingControllerParameters walkingControllerParameters,
                                              BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                              SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(icpPlannerParameters, walkingControllerParameters.getICPOptimizationParameters(), walkingControllerParameters, bipedSupportPolygons,
           icpControlPolygons, contactableFeet, controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public ICPAdjustmentOptimizationController(ICPPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                              WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                              ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                              double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(icpPlannerParameters, walkingControllerParameters, icpOptimizationParameters, bipedSupportPolygons, icpControlPolygons, contactableFeet, controlDT,
            true, yoGraphicsListRegistry);

      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());
      else
         swingSpeedUpEnabled.set(false);

      defaultSwingSplitFraction.set(icpPlannerParameters.getSwingSplitFraction());
      defaultTransferSplitFraction.set(icpPlannerParameters.getTransferSplitFraction());

      transferSplitFractionUnderDisturbance.set(icpOptimizationParameters.getDoubleSupportSplitFractionForBigAdjustment());
      magnitudeForBigAdjustment.set(icpOptimizationParameters.getMagnitudeForBigAdjustment());

      useDifferentSplitRatioForBigAdjustment = icpOptimizationParameters.useDifferentSplitRatioForBigAdjustment();
      minimumTimeOnInitialCMPForBigAdjustment = icpOptimizationParameters.getMinimumTimeOnInitialCMPForBigAdjustment();

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      if (swingSpeedUpEnabled.getBooleanValue() && remainingTimeForSwing < timeRemainingInState.getDoubleValue())
      {
         double speedUpTime = timeRemainingInState.getDoubleValue() - remainingTimeForSwing;
         this.speedUpTime.add(speedUpTime);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForStanding(double initialTime)
   {
      super.initializeForStanding(initialTime);

      speedUpTime.set(0.0);
   }


   @Override
   protected int initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      return super.initializeOnContactChange(initialTime);
   }

   @Override
   protected void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
   }


   /** {@inheritDoc} */
   @Override
   public void compute(double currentTime, FramePoint2D desiredICP, FrameVector2D desiredICPVelocity, FramePoint2D perfectCMP, FramePoint2D currentICP, double omega0)
   {
      controllerTimer.startMeasurement();

      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      this.currentICP.set(currentICP);
      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);

      solutionHandler.getControllerReferenceCMP(referenceCMP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      if (isStanding.getBooleanValue())
         submitSolverTaskConditionsForFeedbackOnlyControl();
      else
         submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

      qpSolverTimer.startMeasurement();
      NoConvergenceException noConvergenceException = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(numberOfFootstepsToConsider, omega0, noConvergenceException);

      controllerTimer.stopMeasurement();
   }

   public double getOptimizedTimeRemaining()
   {
      throw new RuntimeException("This is not implemented in this solver.");
   }
}


