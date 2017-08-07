package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;
import java.util.List;

public class ICPTimingOptimizationController extends AbstractICPOptimizationController
{
   private static final double footVelocityScalarWeight = 0.0;

   private final YoDouble timingAdjustmentWeight = new YoDouble(yoNamePrefix + "TimingAdjustmentWeight", registry);
   private final YoDouble timingAdjustmentRegularizationWeight = new YoDouble(yoNamePrefix + "TimingAdjustmentRegularizationWeight", registry);
   private final YoDouble gradientThresholdForAdjustment = new YoDouble(yoNamePrefix + "GradientThresholdForAdjustment", registry);
   private final YoDouble gradientDescentGain = new YoDouble(yoNamePrefix + "GradientDescentGain", registry);

   private final YoDouble timingAdjustmentAttenuation = new YoDouble(yoNamePrefix + "TimingAdjustmentAttenuation", registry);
   private final YoDouble timingSolutionLowerBound = new YoDouble(yoNamePrefix + "TimingSolutionLowerBound", registry);
   private final YoDouble timingSolutionUpperBound = new YoDouble(yoNamePrefix + "TimingSolutionUpperBound", registry);

   private final YoDouble timingDeadline = new YoDouble(yoNamePrefix + "TimingDeadline", registry);
   private final YoBoolean finishedOnTime = new YoBoolean(yoNamePrefix + "FinishedOnTime", registry);

   private final YoDouble footVelocityWeight = new YoDouble(yoNamePrefix + "FootVelocityWeight", registry);

   private final double variationSizeToComputeTimingGradient;
   private final int maxNumberOfGradientIterations;
   private final int numberOfGradientReductions;
   private final double minimumSwingDuration;
   private static final double percentCostRequiredDecrease = 0.05;

   private final YoDouble referenceSwingDuration = new YoDouble(yoNamePrefix + "ReferenceSwingDuration", registry);

   private final List<YoDouble> swingTimings = new ArrayList<>();
   private final List<YoDouble> timingAdjustments = new ArrayList<>();
   private final List<YoDouble> costToGos = new ArrayList<>();
   private final List<YoDouble> costToGoGradients = new ArrayList<>();

   private final YoInteger numberOfGradientIterations = new YoInteger("numberOfGradientIterations", registry);
   private final YoInteger numberOfGradientReductionIterations = new YoInteger("numberOfGradientReductionIterations", registry);
   private final YoDouble estimatedMinimumCostSwingTime = new YoDouble("estimatedMinimumCostSwingTime", registry);

   private final ICPTimingCostFunctionEstimator costFunctionEstimator = new ICPTimingCostFunctionEstimator();

   private final FramePoint2d currentSwingFootPosition = new FramePoint2d();
   private final FrameVector2d currentSwingFootVelocity = new FrameVector2d();
   private final FrameVector2d requiredSwingFootVelocity = new FrameVector2d();
   private final FrameVector2d requiredFootstepPosition = new FrameVector2d();

   private double previousSwingDurationSolution;

   public ICPTimingOptimizationController(ICPPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                          WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                          ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                          double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(icpPlannerParameters, walkingControllerParameters, icpOptimizationParameters, bipedSupportPolygons, icpControlPolygons, contactableFeet, controlDT,
            false, yoGraphicsListRegistry);

      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      defaultSwingSplitFraction.set(icpPlannerParameters.getSwingSplitFraction());
      defaultTransferSplitFraction.set(icpPlannerParameters.getTransferSplitFraction());

      transferSplitFractionUnderDisturbance.set(icpOptimizationParameters.getDoubleSupportSplitFractionForBigAdjustment());
      magnitudeForBigAdjustment.set(icpOptimizationParameters.getMagnitudeForBigAdjustment());

      useDifferentSplitRatioForBigAdjustment = icpOptimizationParameters.useDifferentSplitRatioForBigAdjustment();
      minimumTimeOnInitialCMPForBigAdjustment = icpOptimizationParameters.getMinimumTimeOnInitialCMPForBigAdjustment();

      timingAdjustmentWeight.set(icpOptimizationParameters.getTimingAdjustmentGradientDescentWeight());
      timingAdjustmentRegularizationWeight.set(icpOptimizationParameters.getTimingAdjustmentGradientDescentRegularizationWeight());
      gradientThresholdForAdjustment.set(icpOptimizationParameters.getGradientThresholdForTimingAdjustment());
      gradientDescentGain.set(icpOptimizationParameters.getGradientDescentGain());
      timingAdjustmentAttenuation.set(icpOptimizationParameters.getTimingAdjustmentAttenuation());
      timingDeadline.set(icpOptimizationParameters.getMaximumDurationForOptimization());

      footVelocityWeight.set(footVelocityScalarWeight);

      variationSizeToComputeTimingGradient = icpOptimizationParameters.getVariationSizeToComputeTimingGradient();
      maxNumberOfGradientIterations = icpOptimizationParameters.getMaximumNumberOfGradientIterations();
      numberOfGradientReductions = icpOptimizationParameters.getMaximumNumberOfGradientReductions();
      minimumSwingDuration = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      for (int i = 0; i < maxNumberOfGradientIterations; i++)
      {
         YoDouble swingTiming = new YoDouble(yoNamePrefix + "SwingTiming" + i, registry);
         YoDouble timingAdjustment = new YoDouble(yoNamePrefix + "TimingAdjustment" + i, registry);
         YoDouble costToGo = new YoDouble(yoNamePrefix + "CostToGo" + i, registry);
         YoDouble costToGoGradient = new YoDouble(yoNamePrefix + "CostToGoGradient" + i, registry);

         swingTimings.add(swingTiming);
         timingAdjustments.add(timingAdjustment);
         costToGos.add(costToGo);
         costToGoGradients.add(costToGoGradient);
      }

      parentRegistry.addChild(registry);
   }


   /** {@inheritDoc} */
   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      // do nothing
   }


   /** {@inheritDoc} */
   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      super.initializeForTransfer(initialTime, transferToSide, omega0);

      referenceSwingDuration.set(swingDurations.get(0).getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      super.initializeForSingleSupport(initialTime, supportSide, omega0);

      referenceSwingDuration.set(swingDurations.get(0).getDoubleValue());
   }













   /** {@inheritDoc} */
   @Override
   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d perfectCMP, FramePoint2d currentICP, double omega0)
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

      NoConvergenceException noConvergenceException;
      qpSolverTimer.startMeasurement();
      if (isStanding.getBooleanValue())
      {
         submitSolverTaskConditionsForFeedbackOnlyControl();
         noConvergenceException = solveQP();
         solver.setPreviousFootstepSolutionFromCurrent();
         solver.setPreviousFeedbackDeltaSolutionFromCurrent();
         numberOfGradientIterations.set(0);
         numberOfGradientReductionIterations.set(0);
         finishedOnTime.set(true);
         costToGos.get(0).set(solver.getCostToGo());
      }
      else if (isInTransfer.getBooleanValue())
      {
         submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);
         noConvergenceException = solveQP();
         solver.setPreviousFootstepSolutionFromCurrent();
         solver.setPreviousFeedbackDeltaSolutionFromCurrent();
         numberOfGradientIterations.set(0);
         numberOfGradientReductionIterations.set(0);
         finishedOnTime.set(true);
         costToGos.get(0).set(solver.getCostToGo());
      }
      else
      {
         noConvergenceException = solveGradientDescent(numberOfFootstepsToConsider, omega0);
      }
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(numberOfFootstepsToConsider, omega0, noConvergenceException);

      previousSwingDurationSolution = swingDurations.get(0).getDoubleValue();

      controllerTimer.stopMeasurement();
   }




   private void resetGradientDescentValues()
   {
      for (int i = 0; i < maxNumberOfGradientIterations; i++)
      {
         costToGos.get(i).setToNaN();
         costToGoGradients.get(i).setToNaN();
         timingAdjustments.get(i).setToNaN();
         swingTimings.get(i).setToNaN();
      }

      costFunctionEstimator.reset();
   }

   private NoConvergenceException solveGradientDescent(int numberOfFootstepsToConsider, double omega0)
   {
      resetGradientDescentValues();

      double timingLowerBound = minimumSwingDuration;
      double timingUpperBound = Double.POSITIVE_INFINITY;
      boolean finishedOnTime = true;

      submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

      NoConvergenceException noConvergenceException = solveQP();
      double solverCostToGo = solver.getCostToGo();
      double timeAdjustmentCostToGo = computeTimeAdjustmentCostToGo();
      double footVelocityCostToGo = computeFootVelocityCostToGo();

      //double costToGoUnvaried = computeTotalCostToGo();
      double costToGoUnvaried = solverCostToGo + timeAdjustmentCostToGo + footVelocityCostToGo;

      double variationSize = variationSizeToComputeTimingGradient;

      if (noConvergenceException != null)
         return noConvergenceException;

      YoDouble swingDuration = swingDurations.get(0);
      swingDuration.add(variationSize);

      submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);
      noConvergenceException = solveQP();

      if (noConvergenceException != null)
         return noConvergenceException;


      double solverCostToGoWithVariation = solver.getCostToGo();
      double timeAdjustmentCostToGoWithVariation = computeTimeAdjustmentCostToGo();
      double footVelocityCostToGoWithVariation = computeFootVelocityCostToGo();

      //double costToGoWithVariation = computeTotalCostToGo();
      double costToGoWithVariation = solverCostToGoWithVariation + timeAdjustmentCostToGoWithVariation + footVelocityCostToGoWithVariation;

      double averageCostToGo = 0.5 * (costToGoWithVariation + costToGoUnvaried);
      double costToGoGradient = (costToGoWithVariation - costToGoUnvaried) / variationSize;
      swingDuration.sub(variationSize);

      costToGos.get(0).set(averageCostToGo);
      costToGoGradients.get(0).set(costToGoGradient);
      swingTimings.get(0).set(swingDuration.getDoubleValue());

      costFunctionEstimator.addPoint(averageCostToGo, costToGoGradient, swingDuration.getDoubleValue());

      int iterationNumber = 0;
      while (Math.abs(costToGoGradient) > gradientThresholdForAdjustment.getDoubleValue())
      {
         // update bounds on the gradient descent
         if (costToGoGradient > 0) // the current gradient is positive, which means that the timing solution must be less
            timingUpperBound = Math.min(swingDuration.getDoubleValue(), timingUpperBound);
         else // the current gradient is negative, which means that the timing solution must be more
            timingLowerBound = Math.max(swingDuration.getDoubleValue(), timingLowerBound);

         // if the cost reduction adjustment requires moving outside the bounds, exit
         if ((MathTools.epsilonEquals(swingDuration.getDoubleValue(), timingLowerBound, 0.0001) && costToGoGradient > 0) ||
               (MathTools.epsilonEquals(swingDuration.getDoubleValue(), timingUpperBound, 0.0001) && costToGoGradient < 0))
            break;

         // estimate time adjustment using gradient based methods
         double timeAdjustment = -gradientDescentGain.getDoubleValue() * costToGoGradient;

         iterationNumber++;
         // exit loop if we've gone too many ticks
         if (iterationNumber >= maxNumberOfGradientIterations)
            break;

         if (controllerTimer.getCurrentTime().getDoubleValue() > timingDeadline.getDoubleValue())
         { // if the controller has taken too long, notify us and break the loop
            finishedOnTime = false;
            break;
         }

         // make sure it doesn't modify the duration outside the bounds
         timeAdjustment = MathTools.clamp(timeAdjustment, timingLowerBound - swingDuration.getDoubleValue(), timingUpperBound - swingDuration.getDoubleValue());
         timingAdjustments.get(iterationNumber).set(timeAdjustment);


         // modify current single support duration
         swingDuration.add(timeAdjustment);

         submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

         noConvergenceException = solveQP();
         if (noConvergenceException != null)
            return noConvergenceException;

         solverCostToGo = solver.getCostToGo();
         timeAdjustmentCostToGo = computeTimeAdjustmentCostToGo();
         footVelocityCostToGo = computeFootVelocityCostToGo();

         //costToGoUnvaried = computeTotalCostToGo();
         costToGoUnvaried = solverCostToGo + timeAdjustmentCostToGo + footVelocityCostToGo;

         int reductionNumber = 0;
         while (costToGoUnvaried >= averageCostToGo + percentCostRequiredDecrease * Math.signum(averageCostToGo) * averageCostToGo)
         {
            // update the bounds
            if (costToGoGradient > 0) // we just decreased the duration, and it caused an increase in cost
               timingLowerBound = Math.max(swingDuration.getDoubleValue(), timingLowerBound);
            else // we just increased the duration, and it caused an increase in cost
               timingUpperBound = Math.min(swingDuration.getDoubleValue(), timingUpperBound);

            // exit loop if we've gone too many ticks
            if (reductionNumber >= numberOfGradientReductions || iterationNumber >= maxNumberOfGradientIterations)
               break;

            if (controllerTimer.getCurrentTime().getDoubleValue() > timingDeadline.getDoubleValue())
            { // if the controller has taken too long, notify us and break the loop
               finishedOnTime = false;
               break;
            }

            // add the current point to the estimator
            costFunctionEstimator.addPoint(costToGoUnvaried, swingDuration.getDoubleValue());

            // the current adjustment causes an increase in cost, so reduce the adjustment
            timeAdjustment = timingAdjustmentAttenuation.getDoubleValue() * timeAdjustment;

            // make sure it doesn't modify the duration outside the bounds
            timeAdjustment = MathTools.clamp(timeAdjustment, swingDuration.getDoubleValue() - timingUpperBound, swingDuration.getDoubleValue() - timingLowerBound);
            timingAdjustments.get(iterationNumber - 1).set(timeAdjustment);
            swingDuration.sub(timeAdjustment);

            // if the cost reduction adjustment requires moving outside the bounds, exit
            if ((MathTools.epsilonEquals(swingDuration.getDoubleValue(), timingLowerBound, 0.0001) && costToGoGradient > 0) ||
                  (MathTools.epsilonEquals(swingDuration.getDoubleValue(), timingUpperBound, 0.0001) && costToGoGradient < 0))
               break;

            // compute new cost at the current time
            submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

            noConvergenceException = solveQP();
            if (noConvergenceException != null)
               return noConvergenceException;

            solverCostToGo = solver.getCostToGo();
            timeAdjustmentCostToGo = computeTimeAdjustmentCostToGo();
            footVelocityCostToGo = computeFootVelocityCostToGo();

            //costToGoUnvaried = computeTotalCostToGo();
            costToGoUnvaried = solverCostToGo + timeAdjustmentCostToGo + footVelocityCostToGo;

            swingTimings.get(iterationNumber).set(swingDuration.getDoubleValue());
            costToGos.get(iterationNumber).set(costToGoUnvaried);
            costToGoGradients.get(iterationNumber).setToNaN();

            iterationNumber++;
            reductionNumber++;
         }

         numberOfGradientReductionIterations.set(reductionNumber);

         // compute gradient at new point
         swingDuration.add(variationSize);

         submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

         noConvergenceException = solveQP();
         if (noConvergenceException != null)
            return noConvergenceException;

         solverCostToGoWithVariation = solver.getCostToGo();
         timeAdjustmentCostToGoWithVariation = computeTimeAdjustmentCostToGo();
         footVelocityCostToGoWithVariation = computeFootVelocityCostToGo();

         //costToGoWithVariation = computeTotalCostToGo();
         costToGoWithVariation = solverCostToGoWithVariation + timeAdjustmentCostToGoWithVariation + footVelocityCostToGoWithVariation;

         averageCostToGo = 0.5 * (costToGoWithVariation + costToGoUnvaried);
         costToGoGradient = (costToGoWithVariation - costToGoUnvaried) / variationSize;
         swingDuration.sub(variationSize);

         swingTimings.get(iterationNumber).set(swingDuration.getDoubleValue());
         costToGos.get(iterationNumber).set(averageCostToGo);
         costToGoGradients.get(iterationNumber).set(costToGoGradient);

         costFunctionEstimator.addPoint(averageCostToGo, costToGoGradient, swingDuration.getDoubleValue());
      }

      timingSolutionLowerBound.set(timingLowerBound);
      timingSolutionUpperBound.set(timingUpperBound);
      this.finishedOnTime.set(finishedOnTime);

      solver.setPreviousFeedbackDeltaSolutionFromCurrent();
      solver.setPreviousFootstepSolutionFromCurrent();

      estimatedMinimumCostSwingTime.set(costFunctionEstimator.getEstimatedCostFunctionSolution());

      numberOfGradientIterations.set(iterationNumber + 1);

      return null;
   }

   public double computeTimeAdjustmentCostToGo()
   {
      double adjustmentWeight = timingAdjustmentWeight.getDoubleValue() * Math.pow(swingDurations.get(0).getDoubleValue() - referenceSwingDuration.getDoubleValue(), 2.0);
      double regularizationWeight = timingAdjustmentRegularizationWeight.getDoubleValue() / controlDT *
            Math.pow(swingDurations.get(0).getDoubleValue() - previousSwingDurationSolution, 2.0);

      return adjustmentWeight + regularizationWeight;
   }

   private final FramePoint tempPoint = new FramePoint();
   private final FrameVector tempVector = new FrameVector();

   public double computeFootVelocityCostToGo()
   {
      RobotSide swingSide = supportSide.getEnumValue().getOppositeSide();
      ReferenceFrame ankleFrame = contactableFeet.get(swingSide).getFrameAfterParentJoint();

      tempPoint.setToZero(ankleFrame);
      footstepSolutions.get(0).getFrameTuple2d(requiredFootstepPosition);

      Twist footTwist = contactableFeet.get(swingSide).getRigidBody().getParentJoint().getFrameAfterJoint().getTwistOfFrame();
      footTwist.getLinearPart(tempVector);

      tempVector.changeFrame(worldFrame);
      currentSwingFootVelocity.setByProjectionOntoXYPlane(tempVector);

      tempPoint.changeFrame(worldFrame);
      currentSwingFootPosition.setByProjectionOntoXYPlane(tempPoint);
      requiredFootstepPosition.changeFrame(worldFrame);

      requiredSwingFootVelocity.set(requiredFootstepPosition);
      requiredSwingFootVelocity.sub(currentSwingFootPosition);
      requiredSwingFootVelocity.scale(1.0 / timeRemainingInState.getDoubleValue());

      currentSwingFootVelocity.sub(requiredSwingFootVelocity);

      double velocityDiffSquared = currentSwingFootVelocity.dot(currentSwingFootVelocity);

      return footVelocityWeight.getDoubleValue() * velocityDiffSquared;
   }

   public double getOptimizedTimeRemaining()
   {
      computeTimeRemainingInState();
      return timeRemainingInState.getDoubleValue();
   }
}
