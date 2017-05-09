package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class ICPAdjustmentOptimizationController extends ICPOptimizationController
{
   protected final boolean useTwoCMPs;
   protected final boolean useFootstepRegularization;
   protected final boolean useFeedbackRegularization;

   private final BooleanYoVariable swingSpeedUpEnabled = new BooleanYoVariable(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final DoubleYoVariable speedUpTime = new DoubleYoVariable(yoNamePrefix + "SpeedUpTime", registry);

   private final DoubleYoVariable finalTransferSplitFraction = new DoubleYoVariable(yoNamePrefix + "FinalTransferSplitFraction", registry);

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);
   private final ExecutionTimer multiplierCalculatorTimer = new ExecutionTimer("icpMultiplierCalculatorTimer", 0.5, registry);
   private final ExecutionTimer referenceValueComputationTimer = new ExecutionTimer("icpReferenceValueComputationTimer", 0.5, registry);

   private final ICPQPOptimizationSolver solver;

   public ICPAdjustmentOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters, bipedSupportPolygons, contactableFeet, controlDT, yoGraphicsListRegistry);

      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      solver = new ICPQPOptimizationSolver(icpOptimizationParameters, totalVertices, COMPUTE_COST_TO_GO);

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

      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());
      else
         swingSpeedUpEnabled.set(false);


      defaultSwingSplitFraction.set(icpPlannerParameters.getSwingDurationAlpha());
      defaultTransferSplitFraction.set(icpPlannerParameters.getTransferDurationAlpha());

      transferSplitFractionUnderDisturbance.set(icpOptimizationParameters.getDoubleSupportSplitFractionForBigAdjustment());
      magnitudeForBigAdjustment.set(icpOptimizationParameters.getMagnitudeForBigAdjustment());

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

   public void setTransferDuration(int stepNumber, double duration)
   {
      int numberOfFootstepsRegistered = upcomingFootsteps.size();
      if (stepNumber < numberOfFootstepsRegistered + 1)
         transferDurations.get(stepNumber).set(duration);
   }


   public void setSwingDuration(int stepNumber, double duration)
   {
      int numberOfFootstepsRegistered = upcomingFootsteps.size();
      if (stepNumber < numberOfFootstepsRegistered)
         swingDurations.get(stepNumber).set(duration);
   }

   /**
    * Allows setting of the transfer duration split fraction (see {@link #transferSplitFractions}) for the specified step number.
    *
    * @param stepNumber step transfer duration split fraction to modify.
    * @param splitFraction new transfer duration split fraction value.
    */
   public void setTransferSplitFraction(int stepNumber, double splitFraction)
   {
      transferSplitFractions.get(stepNumber).set(splitFraction);
   }

   /**
    * Allows setting of the swing duration split fraction (see {@link #swingSplitFractions}) for the specified step number.
    *
    * @param stepNumber step swing duration split fraction to modify.
    * @param splitFraction new swing duration split fraction value.
    */
   public void setSwingSplitFraction(int stepNumber, double splitFraction)
   {
      transferSplitFractions.get(stepNumber).set(splitFraction);
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

   /**
    * Changes the split fraction for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    *
    * @param splitFraction final transfer duration
    */
   public void setFinalTransferSplitFraction(double splitFraction)
   {
      this.finalTransferSplitFraction.set(splitFraction);
   }

   public void setFinalTransferSplitFractionToDefault()
   {
      this.finalTransferSplitFraction.set(defaultTransferSplitFraction.getDoubleValue());
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
      if (swingSpeedUpEnabled.getBooleanValue() && remainingTimeForSwing < timeRemainingInState.getDoubleValue())
      {
         double speedUpTime = timeRemainingInState.getDoubleValue() - remainingTimeForSwing;
         this.speedUpTime.add(speedUpTime);
      }
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

      speedUpTime.set(0.0);
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
   }

   private int initializeOnContactChange(double initialTime)
   {
      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();
      doingBigAdjustment.set(false);

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      this.initialTime.set(initialTime);
      speedUpTime.set(0.0);

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
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
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
      //reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);

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

      inputHandler.computeFinalICPRecursion(stateMultiplierCalculator, finalICPRecursion);
      inputHandler.computeCMPConstantEffects(stateMultiplierCalculator, cmpConstantEffects, beginningOfStateICP.getFrameTuple2d(),
            beginningOfStateICPVelocity.getFrameTuple2d(), useTwoCMPs, isInTransfer.getBooleanValue());
      multiplierCalculatorTimer.stopMeasurement();

      if (isInTransfer.getBooleanValue())
      {
         copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
         //reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);
      }
      else
      {
         copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide.getEnumValue(), solver);
         //reachabilityConstraintHandler.updateReachabilityConstraintForSingleSupport(supportSide.getEnumValue(), solver);
      }


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

   private int clipNumberOfFootstepsToConsiderToProblem(int numberOfFootstepsToConsider)
   {
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, upcomingFootsteps.size());
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

      if (!localUseStepAdjustment || (isInTransfer.getBooleanValue() && !ALLOW_ADJUSTMENT_IN_TRANSFER) || isStanding.getBooleanValue())
         numberOfFootstepsToConsider = 0;

      return numberOfFootstepsToConsider;
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
