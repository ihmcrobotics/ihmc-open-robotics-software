package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.StateMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;
import java.util.List;

public class ICPTimingOptimizationController extends ICPOptimizationController
{
   protected final boolean useTwoCMPs;
   protected final boolean useFootstepRegularization;
   protected final boolean useFeedbackRegularization;

   /** gradient descent variables */
   private final DoubleYoVariable timingAdjustmentWeight = new DoubleYoVariable(yoNamePrefix + "TimingAdjustmentWeight", registry);
   private final DoubleYoVariable gradientThresholdForAdjustment = new DoubleYoVariable(yoNamePrefix + "GradientThresholdForAdjustment", registry);
   private final DoubleYoVariable gradientDescentGain = new DoubleYoVariable(yoNamePrefix + "GradientDescentGain", registry);

   private final DoubleYoVariable timingAdjustmentAttenuation = new DoubleYoVariable(yoNamePrefix + "TimingAdjustmentAttenuation", registry);
   private final DoubleYoVariable timingSolutionLowerBound = new DoubleYoVariable(yoNamePrefix + "TimingSolutionLowerBound", registry);
   private final DoubleYoVariable timingSolutionUpperBound = new DoubleYoVariable(yoNamePrefix + "TimingSolutionUpperBound", registry);

   private final DoubleYoVariable timingDeadline = new DoubleYoVariable(yoNamePrefix + "TimingDeadline", registry);
   private final BooleanYoVariable finishedOnTime = new BooleanYoVariable(yoNamePrefix + "FinishedOnTime", registry);

   private final double variationSizeToComputeTimingGradient;
   private final int maxNumberOfGradientIterations;
   private final int numberOfGradientReductions;
   private final double minimumSwingDuration;
   private static final double percentCostRequiredDecrease = 0.05;


   /** other controller variables */
   private final DoubleYoVariable referenceSwingDuration = new DoubleYoVariable(yoNamePrefix + "ReferenceSwingDuration", registry);

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);
   private final ExecutionTimer multiplierCalculatorTimer = new ExecutionTimer("icpMultiplierCalculatorTimer", 0.5, registry);
   private final ExecutionTimer referenceValueComputationTimer = new ExecutionTimer("icpReferenceValueComputationTimer", 0.5, registry);

   private final ICPQPOptimizationSolver solver;

   private final List<DoubleYoVariable> swingTimings = new ArrayList<>();
   private final List<DoubleYoVariable> timingAdjustments = new ArrayList<>();
   private final List<DoubleYoVariable> costToGos = new ArrayList<>();
   private final List<DoubleYoVariable> costToGoGradients = new ArrayList<>();
   private final IntegerYoVariable numberOfGradientIterations = new IntegerYoVariable("numberOfGradientIterations", registry);
   private final IntegerYoVariable numberOfGradientReductionIterations = new IntegerYoVariable("numberOfGradientReductionIterations", registry);
   private final DoubleYoVariable estimatedMinimumCostSwingTime = new DoubleYoVariable("estimatedMinimumCostSwingTime", registry);

   private final ICPTimingCostFunctionEstimator costFunctionEstimator = new ICPTimingCostFunctionEstimator();

   public ICPTimingOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters, bipedSupportPolygons, contactableFeet, controlDT, yoGraphicsListRegistry);

      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      solver = new ICPQPOptimizationSolver(icpOptimizationParameters, totalVertices, COMPUTE_COST_TO_GO, false);

      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());
      useAngularMomentum.set(icpOptimizationParameters.useAngularMomentum());

      useTwoCMPs = icpPlannerParameters.useTwoCMPsPerSupport();
      useFootstepRegularization = icpOptimizationParameters.useFootstepRegularization();
      useFeedbackRegularization = icpOptimizationParameters.useFeedbackRegularization();

      scaleStepRegularizationWeightWithTime.set(icpOptimizationParameters.scaleStepRegularizationWeightWithTime());
      scaleFeedbackWeightWithGain.set(icpOptimizationParameters.scaleFeedbackWeightWithGain());
      scaleUpcomingStepWeights.set(icpOptimizationParameters.scaleUpcomingStepWeights());

      forwardFootstepWeight.set(icpOptimizationParameters.getForwardFootstepWeight());
      lateralFootstepWeight.set(icpOptimizationParameters.getLateralFootstepWeight());
      footstepRegularizationWeight.set(icpOptimizationParameters.getFootstepRegularizationWeight());
      feedbackForwardWeight.set(icpOptimizationParameters.getFeedbackForwardWeight());
      feedbackLateralWeight.set(icpOptimizationParameters.getFeedbackLateralWeight());
      feedbackRegularizationWeight.set(icpOptimizationParameters.getFeedbackRegularizationWeight());
      feedbackOrthogonalGain.set(icpOptimizationParameters.getFeedbackOrthogonalGain());
      feedbackParallelGain.set(icpOptimizationParameters.getFeedbackParallelGain());
      dynamicRelaxationWeight.set(icpOptimizationParameters.getDynamicRelaxationWeight());
      angularMomentumMinimizationWeight.set(icpOptimizationParameters.getAngularMomentumMinimizationWeight());

      minimumTimeRemaining.set(icpOptimizationParameters.getMinimumTimeRemaining());

      defaultSwingSplitFraction.set(icpPlannerParameters.getSwingDurationAlpha());
      defaultTransferSplitFraction.set(icpPlannerParameters.getTransferDurationAlpha());


      transferSplitFractionUnderDisturbance.set(icpOptimizationParameters.getDoubleSupportSplitFractionForBigAdjustment());
      magnitudeForBigAdjustment.set(icpOptimizationParameters.getMagnitudeForBigAdjustment());

      timingAdjustmentWeight.set(icpOptimizationParameters.getTimingAdjustmentGradientDescentWeight());
      gradientThresholdForAdjustment.set(icpOptimizationParameters.getGradientThresholdForTimingAdjustment());
      gradientDescentGain.set(icpOptimizationParameters.getGradientDescentGain());
      timingAdjustmentAttenuation.set(icpOptimizationParameters.getTimingAdjustmentAttenuation());
      timingDeadline.set(icpOptimizationParameters.getMaximumDurationForOptimization());

      variationSizeToComputeTimingGradient = icpOptimizationParameters.getVariationSizeToComputeTimingGradient();
      maxNumberOfGradientIterations = icpOptimizationParameters.getMaximumNumberOfGradientIterations();
      numberOfGradientReductions = icpOptimizationParameters.getMaximumNumberOfGradientReductions();
      minimumSwingDuration = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         upcomingFootstepLocations.add(new YoFramePoint2d(yoNamePrefix + "UpcomingFootstepLocation" + i, worldFrame, registry));
         footstepSolutions.add(new YoFramePoint2d(yoNamePrefix + "FootstepSolutionLocation" + i, worldFrame, registry));
         unclippedFootstepSolutions.add(new FramePoint2d(worldFrame));

         DoubleYoVariable swingDuration = new DoubleYoVariable(yoNamePrefix + "SwingDuration" + i, registry);
         swingDuration.setToNaN();
         swingDurations.add(swingDuration);
         DoubleYoVariable transferDuration = new DoubleYoVariable(yoNamePrefix + "TransferDuration" + i, registry);
         transferDuration.setToNaN();
         transferDurations.add(transferDuration);

         DoubleYoVariable transferSplitFraction = new DoubleYoVariable(yoNamePrefix + "TransferSplitFraction" + i,
               "Repartition of the transfer duration around the entry corner point.", registry);
         transferSplitFraction.setToNaN();
         transferSplitFractions.add(transferSplitFraction);
         DoubleYoVariable swingSplitFraction = new DoubleYoVariable(yoNamePrefix + "SwingSplitFraction" + i,
               "Repartition of the transfer duration around the entry corner point.", registry);
         swingSplitFraction.setToNaN();
         swingSplitFractions.add(swingSplitFraction);
      }

      for (int i = 0; i < maxNumberOfGradientIterations; i++)
      {
         DoubleYoVariable swingTiming = new DoubleYoVariable(yoNamePrefix + "SwingTiming" + i, registry);
         DoubleYoVariable timingAdjustment = new DoubleYoVariable(yoNamePrefix + "TimingAdjustment" + i, registry);
         DoubleYoVariable costToGo = new DoubleYoVariable(yoNamePrefix + "CostToGo" + i, registry);
         DoubleYoVariable costToGoGradient = new DoubleYoVariable(yoNamePrefix + "CostToGoGradient" + i, registry);

         swingTimings.add(swingTiming);
         timingAdjustments.add(timingAdjustment);
         costToGos.add(costToGo);
         costToGoGradients.add(costToGoGradient);
      }

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry, VISUALIZE);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition beginningOfStateICP = new YoGraphicPosition(yoNamePrefix + "BeginningOfStateICP`", this.beginningOfStateICP, 0.01, YoAppearance.MidnightBlue(),
            YoGraphicPosition.GraphicType.SOLID_BALL);

      artifactList.add(beginningOfStateICP.createArtifact());
      artifactList.setVisible(visualize);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /**
    * Sets the weight on tracking the reference footstep locations in the optimization.
    *
    * @param forwardWeight tracking weight in the forward direction.
    * @param lateralWeight tracking weight in the lateral direction.
    */
   public void setFootstepWeights(double forwardWeight, double lateralWeight)
   {
      forwardFootstepWeight.set(forwardWeight);
      lateralFootstepWeight.set(lateralWeight);
   }

   /**
    * Sets the weight on minimizing the feedback action for the optimization.
    *
    * @param forwardWeight feedback minimization weight in the forward direction.
    * @param lateralWeight feedback minimization weight in the lateral direction.
    */
   public void setFeedbackWeights(double forwardWeight, double lateralWeight)
   {
      feedbackForwardWeight.set(forwardWeight);
      feedbackLateralWeight.set(lateralWeight);
   }

   /**
    * Clear footstep and timing information making the ICP planner ready to be reinitialized with
    * new footsteps.
    * <p>
    * Don't forget to call this method before registering a new set of footsteps.
    * </p>
    */
   public void clearPlan()
   {
      upcomingFootsteps.clear();

      inputHandler.clearPlan();
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         upcomingFootstepLocations.get(i).setToZero();

         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();

         swingSplitFractions.get(i).setToNaN();
         transferSplitFractions.get(i).setToNaN();
      }
   }

   @Override
   public void setTransferDuration(int stepNumber, double duration)
   {
      //// TODO: 5/8/17  
   }

   @Override
   public void setSwingDuration(int stepNumber, double duration)
   {
      swingDurations.get(stepNumber).set(duration);
   }

   @Override
   public void setTransferSplitFraction(int stepNumber, double splitFraction)
   {
      //// TODO: 5/8/17
   }

   @Override
   public void setSwingSplitFraction(int stepNumber, double splitFraction)
   {
      //// TODO: 5/8/17
   }

   /**
    * Changes the duration for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    *
    * @param finalTransferDuration final transfer duration
    */
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration.set(finalTransferDuration);
   }

   @Override
   public void setFinalTransferSplitFraction(double splitFraction)
   {
      //// TODO: 5/8/17  

   }

   @Override
   public void setFinalTransferSplitFractionToDefault()
   {
      //// TODO: 5/8/17  

   }

   /**
    * Registers an additional footstep to consider in the controller.
    * <p>
    * Footsteps have to be registered before initializing the controller.
    * </p>
    * <p>
    * The reference to {@code footstep} is saved internally.
    * </p>
    *
    * @param footstep the new footstep to be queued to the current list of footsteps. Not modified.
    * @param timing the timings to use when performing the footstep. Not modified.
    */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
         {
            int footstepIndex = upcomingFootsteps.size();
            upcomingFootsteps.add(footstep);
            RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(footstep.getRobotSide());
            footstep.getAnklePosition2d(tempPoint2d, ankleToSole);
            upcomingFootstepLocations.get(footstepIndex).set(tempPoint2d);
            inputHandler.addFootstepToPlan(footstep);

            footstepSolutions.get(footstepIndex).set(tempPoint2d);
            unclippedFootstepSolutions.get(footstepIndex).set(tempPoint2d);

            swingDurations.get(footstepIndex).set(timing.getSwingTime());
            transferDurations.get(footstepIndex).set(timing.getTransferTime());
            swingSplitFractions.get(footstepIndex).set(defaultSwingSplitFraction.getDoubleValue());
            transferSplitFractions.get(footstepIndex).set(defaultTransferSplitFraction.getDoubleValue());
         }
         else
         {
            PrintTools.warn(this, "Received bad footstep: " + footstep);
         }
      }
   }

   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
   }

   /**
    * Initializes the controller to smoothly re-center the ICP in the support polygon preparing the
    * robot for standing.
    * <p>
    * Does not use the recursive dynamics, but simply holds the current position.
    * </p>
    * <p>
    * This method is typically useful when done with a walking sequence so the robot smoothly
    * terminates its last transfer.
    * </p>
    * <p>
    * Call {@link #setFinalTransferDuration(double)} beforehand to change the time taken to
    * re-center the ICP.
    * </p>
    *
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();
      doingBigAdjustment.set(false);

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);

      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferSplitFractions.get(0).set(defaultTransferSplitFraction.getDoubleValue());
   }

   /**
    * Prepares the ICP controller for a transfer phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} before calling this method.
    * </p>
    *
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.transferToSide.set(transferToSide);
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      isInTransfer.set(true);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferSplitFractions.get(numberOfFootstepRegistered).set(defaultTransferSplitFraction.getDoubleValue());

      int numberOfFootstepsToConsider = initializeOnContactChange(initialTime);

      stateMultiplierCalculator.initializeForDoubleSupport();
      stateMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, numberOfFootstepRegistered, useTwoCMPs, omega0);

      inputHandler.initializeForDoubleSupport(stateMultiplierCalculator, numberOfFootstepsToConsider, upcomingFootstepLocations, isStanding.getBooleanValue(),
            useTwoCMPs, transferToSide, omega0);
      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);

      referenceSwingDuration.set(swingDurations.get(0).getDoubleValue());
   }

   /**
    * Prepares the ICP controller for a single support phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} before calling this method.
    * </p>
    *
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInTransfer.set(false);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferSplitFractions.get(numberOfFootstepRegistered).set(defaultTransferSplitFraction.getDoubleValue());

      int numberOfFootstepsToConsider = initializeOnContactChange(initialTime);

      stateMultiplierCalculator.initializeForSingleSupport();
      stateMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, numberOfFootstepRegistered, useTwoCMPs, omega0);

      inputHandler.initializeForSingleSupport(stateMultiplierCalculator, numberOfFootstepsToConsider, upcomingFootstepLocations, useTwoCMPs, supportSide, omega0);
      copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForSingleSupport(supportSide, solver);

      referenceSwingDuration.set(swingDurations.get(0).getDoubleValue());
   }

   private int initializeOnContactChange(double initialTime)
   {
      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();
      doingBigAdjustment.set(false);

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      this.initialTime.set(initialTime);

      setBeginningOfStateICP(solutionHandler.getControllerReferenceICP(), solutionHandler.getControllerReferenceICPVelocity());

      if (useFootstepRegularization)
      {
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
            solver.resetFootstepRegularization(i, upcomingFootstepLocations.get(i).getFrameTuple2d());
      }

      solver.resetOnContactChange();

      return numberOfFootstepsToConsider;
   }

   public void setBeginningOfStateICP(FramePoint2d beginningOfStateICP, FrameVector2d beginningOfStateICPVelocity)
   {
      this.beginningOfStateICP.set(beginningOfStateICP);
      this.beginningOfStateICPVelocity.set(beginningOfStateICPVelocity);
   }





   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
   }

   private void computeTimeRemainingInState()
   {
      if (isStanding.getBooleanValue())
      {
         timeRemainingInState.set(0.0);
      }
      else
      {
         if (isInTransfer.getBooleanValue())
            timeRemainingInState.set(transferDurations.get(0).getDoubleValue() - timeInCurrentState.getDoubleValue());
         else
            timeRemainingInState.set(swingDurations.get(0).getDoubleValue() - timeInCurrentState.getDoubleValue());
      }
   }

   private void scaleStepRegularizationWeightWithTime()
   {
      if (scaleStepRegularizationWeightWithTime.getBooleanValue())
      {
         double alpha = Math.max(timeRemainingInState.getDoubleValue(), minimumTimeRemaining.getDoubleValue()) / swingDurations.get(0).getDoubleValue();
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue() / alpha);
      }
      else
      {
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue());
      }
   }

   private void scaleFeedbackWeightWithGain()
   {
      ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();

      ICPOptimizationControllerHelper.transformWeightsToWorldFrame(tempVector2d, feedbackForwardWeight, feedbackLateralWeight, soleFrame);
      scaledFeedbackWeight.set(tempVector2d);

      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         ICPOptimizationControllerHelper.transformFeedbackGains(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);
         scaledFeedbackWeight.scale(1.0 / tempVector2d.length());
      }
   }





   private void submitSolverTaskConditionsForFeedbackOnlyControl()
   {
      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);

      solver.resetFootstepConditions();

      submitFeedbackTaskConditionsToSolver();
      submitAngularMomentumTaskConditionsToSolver();

      finalICPRecursion.set(desiredICP);
      cmpConstantEffects.setToZero();
   }

   private int submitSolverTaskConditionsForSteppingControl(int numberOfFootstepsToConsider, double omega0)
   {
      multiplierCalculatorTimer.startMeasurement();
      stateMultiplierCalculator.computeCurrentMultipliers(numberOfFootstepsToConsider, timeInCurrentState.getDoubleValue(), useTwoCMPs,
            isInTransfer.getBooleanValue(), omega0);

      // // TODO: 5/1/17 do these change from problem to problem?
      inputHandler.computeFinalICPRecursion(stateMultiplierCalculator, finalICPRecursion);
      inputHandler.computeCMPConstantEffects(stateMultiplierCalculator, cmpConstantEffects, beginningOfStateICP.getFrameTuple2d(),
            beginningOfStateICPVelocity.getFrameTuple2d(), useTwoCMPs, isInTransfer.getBooleanValue());
      multiplierCalculatorTimer.stopMeasurement();

      if (isInTransfer.getBooleanValue())
         copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      else
         copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide.getEnumValue(), solver);

      solver.resetFootstepConditions();

      if (localUseStepAdjustment && (!isInTransfer.getBooleanValue() || ALLOW_ADJUSTMENT_IN_TRANSFER))
         submitFootstepTaskConditionsToSolver(numberOfFootstepsToConsider);

      submitFeedbackTaskConditionsToSolver();
      submitAngularMomentumTaskConditionsToSolver();

      return numberOfFootstepsToConsider;
   }

   private void submitFeedbackTaskConditionsToSolver()
   {
      ICPOptimizationControllerHelper.transformFeedbackGains(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);

      double dynamicRelaxationWeight = this.dynamicRelaxationWeight.getDoubleValue();
      if (!localUseStepAdjustment)
         dynamicRelaxationWeight = dynamicRelaxationWeight / dynamicRelaxationDoubleSupportWeightModifier;

      solver.resetFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), tempVector2d.getX(), tempVector2d.getY(),
            dynamicRelaxationWeight);

      if (useFeedbackRegularization)
         solver.setFeedbackRegularizationWeight(feedbackRegularizationWeight.getDoubleValue() / controlDT);
   }

   private void submitAngularMomentumTaskConditionsToSolver()
   {
      double angularMomentumMinimizationWeight = this.angularMomentumMinimizationWeight.getDoubleValue();

      solver.resetAngularMomentumConditions();
      solver.setAngularMomentumConditions(angularMomentumMinimizationWeight, useAngularMomentum.getBooleanValue());
   }

   private void submitFootstepTaskConditionsToSolver(int numberOfFootstepsToConsider)
   {
      for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
      {
         ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();
         ICPOptimizationControllerHelper.transformWeightsToWorldFrame(tempVector2d, forwardFootstepWeight, lateralFootstepWeight, soleFrame);
         scaledFootstepWeights.set(tempVector2d);

         if (localScaleUpcomingStepWeights)
            scaledFootstepWeights.scale(1.0 / (footstepIndex + 1));

         double footstepRecursionMultiplier = stateMultiplierCalculator.getFootstepRecursionMultiplier(useTwoCMPs, footstepIndex);

         solver.setFootstepAdjustmentConditions(footstepIndex, footstepRecursionMultiplier, scaledFootstepWeights.getX(), scaledFootstepWeights.getY(),
               upcomingFootstepLocations.get(footstepIndex).getFrameTuple2d());
      }

      if (useFootstepRegularization)
         solver.setFootstepRegularizationWeight(scaledFootstepRegularizationWeight.getDoubleValue() / controlDT);
   }










   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP, double omega0)
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

      controllerTimer.stopMeasurement();
   }

   private int clipNumberOfFootstepsToConsiderToProblem(int numberOfFootstepsToConsider)
   {
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, upcomingFootsteps.size());
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

      if (!localUseStepAdjustment || (isInTransfer.getBooleanValue() && !ALLOW_ADJUSTMENT_IN_TRANSFER) || isStanding.getBooleanValue())
         numberOfFootstepsToConsider = 0;

      return numberOfFootstepsToConsider;
   }


   private NoConvergenceException solveGradientDescent(int numberOfFootstepsToConsider, double omega0)
   {
      for (int i = 0; i < maxNumberOfGradientIterations; i++)
      {
         costToGos.get(i).setToNaN();
         costToGoGradients.get(i).setToNaN();
         timingAdjustments.get(i).setToNaN();
         swingTimings.get(i).setToNaN();
      }

      costFunctionEstimator.reset();
      double timingLowerBound = minimumSwingDuration;
      double timingUpperBound = Double.POSITIVE_INFINITY;
      boolean finishedOnTime = true;

      submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

      NoConvergenceException noConvergenceException = solveQP();
      double costToGoUnvaried = computeTotalCostToGo();
      double variationSize = variationSizeToComputeTimingGradient;

      if (noConvergenceException != null)
         return noConvergenceException;

      DoubleYoVariable swingDuration = swingDurations.get(0);
      swingDuration.add(variationSize);

      submitSolverTaskConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);
      noConvergenceException = solveQP();

      if (noConvergenceException != null)
         return noConvergenceException;

      double costToGoWithVariation = computeTotalCostToGo();
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

         costToGoUnvaried = computeTotalCostToGo();

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

            costToGoUnvaried = computeTotalCostToGo();

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

         costToGoWithVariation = computeTotalCostToGo();

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

   public double computeTotalCostToGo()
   {
      double solverCostToGo = solver.getCostToGo();
      double timingCostToGo = timingAdjustmentWeight.getDoubleValue() * Math.pow(swingDurations.get(0).getDoubleValue() - referenceSwingDuration.getDoubleValue(), 2.0);

      return solverCostToGo + timingCostToGo;
   }


   private NoConvergenceException solveQP()
   {
      NoConvergenceException noConvergenceException = null;
      try
      {
         solver.compute(finalICPRecursion, cmpConstantEffects, currentICP, referenceCMP);
      }
      catch (NoConvergenceException e)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            e.printStackTrace();
            PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      return noConvergenceException;
   }

   private void extractSolutionsFromSolver(int numberOfFootstepsToConsider, double omega0, NoConvergenceException noConvergenceException)
   {
      // don't pole the new solutions if there's a no convergence exception
      if (noConvergenceException == null)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment)
            solutionHandler.extractFootstepSolutions(footstepSolutions, unclippedFootstepSolutions, upcomingFootstepLocations, upcomingFootsteps,
                  numberOfFootstepsToConsider, solver);

         solver.getCMPFeedbackDifference(tempVector2d);
         controllerFeedbackCMPDelta.set(tempVector2d);

         if (COMPUTE_COST_TO_GO)
            solutionHandler.updateCostsToGo(solver);
      }

      referenceValueComputationTimer.startMeasurement();
      if (isStanding.getBooleanValue())
      {
         solutionHandler.setReferenceValues(desiredICP, desiredICPVelocity, omega0);
      }
      else
      {
         solutionHandler.computeReferenceValuesFromSolution(unclippedFootstepSolutions, inputHandler, stateMultiplierCalculator, beginningOfStateICP,
               beginningOfStateICPVelocity, omega0, numberOfFootstepsToConsider);

         if (DEBUG)
            solutionHandler.computeNominalValues(upcomingFootstepLocations, inputHandler, stateMultiplierCalculator, beginningOfStateICP,
                  beginningOfStateICPVelocity, omega0, numberOfFootstepsToConsider);

         if (useDifferentSplitRatioForBigAdjustment && !isInTransfer.getBooleanValue())
            computeUpcomingDoubleSupportSplitFraction(numberOfFootstepsToConsider, omega0);
      }
      referenceValueComputationTimer.stopMeasurement();

      solver.getDynamicRelaxation(tempPoint2d);
      dynamicRelaxation.set(tempPoint2d);

      solver.getCMPDifferenceFromCoP(tempPoint2d);
      cmpCoPDifferenceSolution.set(tempPoint2d);

      icpError.set(currentICP);
      icpError.sub(solutionHandler.getControllerReferenceICP());

      controllerFeedbackCMPDelta.getFrameTuple2d(tempVector2d);
      solutionHandler.getControllerReferenceCMP(tempPoint2d);
      controllerFeedbackCMP.set(tempPoint2d);
      controllerFeedbackCMP.add(tempVector2d);
   }


   private void computeUpcomingDoubleSupportSplitFraction(int numberOfFootstepsToConsider, double omega0)
   {
      double footstepAdjustmentSize = solutionHandler.getFootstepAdjustment().length();

      double minimumDoubleSupportSplitFraction = minimumTimeOnInitialCMPForBigAdjustment / transferDurations.get(0).getDoubleValue();
      minimumDoubleSupportSplitFraction = Math.max(minimumDoubleSupportSplitFraction, transferSplitFractionUnderDisturbance.getDoubleValue());

      if (footstepAdjustmentSize > magnitudeForBigAdjustment.getDoubleValue() && !doingBigAdjustment.getBooleanValue())
      {
         doingBigAdjustment.set(true);
         transferSplitFractions.get(1).set(minimumDoubleSupportSplitFraction);
         stateMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, upcomingFootsteps.size(), useTwoCMPs, omega0);
      }
   }

   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider.getIntegerValue();
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controllerFeedbackCMP.getFrameTuple2d(desiredCMPToPack);
   }

   public void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack)
   {
      footstepSolutions.get(footstepIndex).getFrameTuple2d(footstepSolutionToPack);
   }

   public boolean wasFootstepAdjusted()
   {
      return solutionHandler.wasFootstepAdjusted();
   }

   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getBooleanValue();
   }
}
