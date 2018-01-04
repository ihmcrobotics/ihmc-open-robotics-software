package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationController implements ICPOptimizationControllerInterface
{
   private static final boolean VISUALIZE = false;
   private static final boolean DEBUG = false;
   private static final boolean COMPUTE_COST_TO_GO = false;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean useAngularMomentum = new YoBoolean(yoNamePrefix + "UseAngularMomentum", registry);

   private final YoBoolean scaleStepRegularizationWeightWithTime = new YoBoolean(yoNamePrefix + "ScaleStepRegularizationWeightWithTime", registry);
   private final YoBoolean scaleFeedbackWeightWithGain = new YoBoolean(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry);

   private final YoBoolean isStanding = new YoBoolean(yoNamePrefix + "IsStanding", registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);
   private final YoDouble transferDuration = new YoDouble(yoNamePrefix + "TransferDuration", registry);
   private final YoDouble nextTransferDuration = new YoDouble(yoNamePrefix + "NextTransferDuration", registry);
   private final YoDouble finalTransferDuration = new YoDouble(yoNamePrefix + "FinalTransferDuration", registry);

   private final YoDouble transferDurationSplitFraction = new YoDouble(yoNamePrefix + "TransferDurationSplitFraction", registry);

   private final YoEnum<RobotSide> transferToSide = new YoEnum<>(yoNamePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final YoEnum<RobotSide> supportSide = new YoEnum<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);
   private final YoDouble minimumTimeRemaining = new YoDouble(yoNamePrefix + "MinimumTimeRemaining", registry);

   private final YoFrameVector2d icpError = new YoFrameVector2d(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoFramePoint2d feedbackCMP = new YoFramePoint2d(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   private final YoFramePoint2d yoPerfectCMP = new YoFramePoint2d(yoNamePrefix + "PerfectCMP", worldFrame, registry);
   private final YoFramePoint2d predictedEndOfStateICP = new YoFramePoint2d(yoNamePrefix + "PredictedEndOfStateICP", worldFrame, registry);

   private final YoFrameVector2d feedbackCoPDelta = new YoFrameVector2d(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   private final YoFrameVector2d cmpCoPDifferenceSolution = new YoFrameVector2d(yoNamePrefix + "CMPCoPDifferenceSolution", "", worldFrame, registry);
   private final YoFramePoint2d feedbackCMPDelta = new YoFramePoint2d(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);

   private final List<Footstep> upcomingFootsteps = new ArrayList<>();

   private final YoFramePoint2d footstepSolution = new YoFramePoint2d(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2d upcomingFootstepLocation = new YoFramePoint2d(yoNamePrefix + "UpcomingFootstepLocation", worldFrame, registry);
   private final FramePoint2D unclippedFootstepSolution = new FramePoint2D();

   private final YoDouble footstepAdjustmentSafetyFactor = new YoDouble(yoNamePrefix + "FootstepAdjustmentSafetyFactor", registry);
   private final YoDouble forwardFootstepWeight = new YoDouble(yoNamePrefix + "ForwardFootstepWeight", registry);
   private final YoDouble lateralFootstepWeight = new YoDouble(yoNamePrefix + "LateralFootstepWeight", registry);
   private final YoFramePoint2d footstepWeights = new YoFramePoint2d(yoNamePrefix + "FootstepWeights", worldFrame, registry);

   private final YoDouble feedbackForwardWeight = new YoDouble(yoNamePrefix + "FeedbackForwardWeight", registry);
   private final YoDouble feedbackLateralWeight = new YoDouble(yoNamePrefix + "FeedbackLateralWeight", registry);
   private final YoFramePoint2d scaledFeedbackWeight = new YoFramePoint2d(yoNamePrefix + "ScaledFeedbackWeight", worldFrame, registry);

   private final YoDouble maxAllowedDistanceCMPSupport = new YoDouble(yoNamePrefix + "MaxAllowedDistanceCMPSupport", registry);
   private final YoDouble safeCoPDistanceToEdge = new YoDouble(yoNamePrefix + "SafeCoPDistanceToEdge", registry);

   private final YoDouble footstepRegularizationWeight = new YoDouble(yoNamePrefix + "FootstepRegularizationWeight", registry);
   private final YoDouble feedbackRegularizationWeight = new YoDouble(yoNamePrefix + "FeedbackRegularizationWeight", registry);
   private final YoDouble scaledFootstepRegularizationWeight = new YoDouble(yoNamePrefix + "ScaledFootstepRegularizationWeight", registry);
   private final YoDouble dynamicsObjectiveWeight = new YoDouble(yoNamePrefix + "DynamicsObjectiveWeight", registry);

   private final YoDouble angularMomentumMinimizationWeight = new YoDouble(yoNamePrefix + "AngularMomentumMinimizationWeight", registry);
   private final YoDouble scaledAngularMomentumMinimizationWeight = new YoDouble(yoNamePrefix + "ScaledAngularMomentumMinimizationWeight", registry);
   private final YoDouble cumulativeAngularMomentum = new YoDouble(yoNamePrefix + "CumulativeAngularMomentum", registry);

   private final YoBoolean limitReachabilityFromAdjustment = new YoBoolean(yoNamePrefix + "LimitReachabilityFromAdjustment", registry);

   private final YoBoolean useICPControlPolygons = new YoBoolean(yoNamePrefix + "UseICPControlPolygons", registry);

   private final YoDouble feedbackOrthogonalGain = new YoDouble(yoNamePrefix + "FeedbackOrthogonalGain", registry);
   private final YoDouble feedbackParallelGain = new YoDouble(yoNamePrefix + "FeedbackParallelGain", registry);

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "FootstepMultiplier", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final YoBoolean useAngularMomentumIntegrator = new YoBoolean(yoNamePrefix + "UseAngularMomentumIntegrator", registry);
   private final YoDouble angularMomentumIntegratorGain = new YoDouble(yoNamePrefix + "AngularMomentumIntegratorGain", registry);
   private final YoDouble angularMomentumIntegratorLeakRatio = new YoDouble(yoNamePrefix + "AngularMomentumIntegratorLeakRatio", registry);

   private final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   private final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   private final ICPOptimizationSolutionHandler solutionHandler;
   private final ICPOptimizationQPSolver solver;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final boolean useFootstepRegularization;
   private final boolean useFeedbackRegularization;

   private boolean localUseStepAdjustment;

   private final FramePoint2D tempPoint2d = new FramePoint2D();
   private final FrameVector2D tempVector2d = new FrameVector2D();

   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D perfectCMP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();

   private final double controlDT;
   private final double dynamicsObjectiveDoubleSupportWeightModifier;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                    double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet,
           controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                    BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.contactableFeet = contactableFeet;

      dynamicsObjectiveDoubleSupportWeightModifier = icpOptimizationParameters.getDynamicsObjectiveDoubleSupportWeightModifier();

      useFootstepRegularization = icpOptimizationParameters.useFootstepRegularization();
      useFeedbackRegularization = icpOptimizationParameters.useFeedbackRegularization();

      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());
      useAngularMomentum.set(icpOptimizationParameters.useAngularMomentum());

      scaleStepRegularizationWeightWithTime.set(icpOptimizationParameters.scaleStepRegularizationWeightWithTime());
      scaleFeedbackWeightWithGain.set(icpOptimizationParameters.scaleFeedbackWeightWithGain());

      footstepAdjustmentSafetyFactor.set(icpOptimizationParameters.getFootstepAdjustmentSafetyFactor());
      forwardFootstepWeight.set(icpOptimizationParameters.getForwardFootstepWeight());
      lateralFootstepWeight.set(icpOptimizationParameters.getLateralFootstepWeight());
      footstepRegularizationWeight.set(icpOptimizationParameters.getFootstepRegularizationWeight());

      feedbackForwardWeight.set(icpOptimizationParameters.getFeedbackForwardWeight());
      feedbackLateralWeight.set(icpOptimizationParameters.getFeedbackLateralWeight());
      feedbackRegularizationWeight.set(icpOptimizationParameters.getFeedbackRegularizationWeight());
      feedbackOrthogonalGain.set(icpOptimizationParameters.getFeedbackOrthogonalGain());
      feedbackParallelGain.set(icpOptimizationParameters.getFeedbackParallelGain());

      dynamicsObjectiveWeight.set(icpOptimizationParameters.getDynamicsObjectiveWeight());

      angularMomentumMinimizationWeight.set(icpOptimizationParameters.getAngularMomentumMinimizationWeight());
      scaledAngularMomentumMinimizationWeight.set(icpOptimizationParameters.getAngularMomentumMinimizationWeight());

      limitReachabilityFromAdjustment.set(icpOptimizationParameters.getLimitReachabilityFromAdjustment());

      transferDurationSplitFraction.set(icpOptimizationParameters.getTransferSplitFraction());

      useAngularMomentumIntegrator.set(icpOptimizationParameters.getUseAngularMomentumIntegrator());
      angularMomentumIntegratorGain.set(icpOptimizationParameters.getAngularMomentumIntegratorGain());
      angularMomentumIntegratorLeakRatio.set(icpOptimizationParameters.getAngularMomentumIntegratorLeakRatio());

      useICPControlPolygons.set(icpOptimizationParameters.getUseICPControlPolygons());

      safeCoPDistanceToEdge.set(icpOptimizationParameters.getSafeCoPDistanceToEdge());
      if (walkingControllerParameters != null)
         maxAllowedDistanceCMPSupport.set(walkingControllerParameters.getMaxAllowedDistanceCMPSupport());
      else
         maxAllowedDistanceCMPSupport.setToNaN();

      minimumTimeRemaining.set(icpOptimizationParameters.getMinimumTimeRemaining());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRegularizationAutomatically = true;
      solver = new ICPOptimizationQPSolver(icpOptimizationParameters, totalVertices, COMPUTE_COST_TO_GO, updateRegularizationAutomatically);
      solver.setConsiderAngularMomentumInAdjustment(icpOptimizationParameters.considerAngularMomentumInAdjustment());
      solver.setConsiderFeedbackInAdjustment(icpOptimizationParameters.considerFeedbackInAdjustment());

      solutionHandler = new ICPOptimizationSolutionHandler(icpOptimizationParameters, DEBUG, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons, useICPControlPolygons);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, yoNamePrefix, VISUALIZE,
                                                                                       registry, yoGraphicsListRegistry);


      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition predictedEndOfStateICP = new YoGraphicPosition(yoNamePrefix + "PredictedEndOfStateICP", this.predictedEndOfStateICP, 0.005, YoAppearance.MidnightBlue(),
                                                                       YoGraphicPosition.GraphicType.BALL);
      solutionHandler.setupVisualizers(artifactList);

      artifactList.add(predictedEndOfStateICP.createArtifact());
      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void clearPlan()
   {
      upcomingFootsteps.clear();
      upcomingFootstepLocation.setToZero();

      transferDuration.setToNaN();
      swingDuration.setToNaN();
      nextTransferDuration.setToNaN();
   }

   @Override
   public void setTransferDuration(double duration)
   {
      transferDuration.set(duration);
   }

   @Override
   public void setSwingDuration(double duration)
   {
      swingDuration.set(duration);
   }

   @Override
   public void setNextTransferDuration(double duration)
   {
      nextTransferDuration.set(duration);
   }

   @Override
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration.set(finalTransferDuration);
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
         {
            if (upcomingFootsteps.size() == 0)
            {
               footstep.getPosition2d(tempPoint2d);
               upcomingFootstepLocation.set(tempPoint2d);

               footstepSolution.set(tempPoint2d);
               unclippedFootstepSolution.set(tempPoint2d);

               swingDuration.set(timing.getSwingTime());
               transferDuration.set(timing.getTransferTime());
            }
            else if (upcomingFootsteps.size() == 1)
            {
               nextTransferDuration.set(timing.getTransferTime());
            }

            upcomingFootsteps.add(footstep);
         }
         else
         {
            PrintTools.warn(this, "Received bad footstep: " + footstep);
         }
      }
   }

   private void updateYoFootsteps()
   {
      if (upcomingFootsteps.size() > 0)
      {
         upcomingFootsteps.get(0).getPosition2d(tempPoint2d);
         upcomingFootstepLocation.set(tempPoint2d);
      }
   }

   @Override
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInDoubleSupport.set(true);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);

      transferDuration.set(finalTransferDuration.getDoubleValue());

      speedUpTime.set(0.0);
   }

   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.transferToSide.set(transferToSide);
      isInDoubleSupport.set(true);

      if (upcomingFootsteps.size() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

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

      if (upcomingFootsteps.size() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForSingleSupport(supportSide, solver);
   }

   private void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      this.initialTime.set(initialTime);

      if (useFootstepRegularization)
      {
         upcomingFootsteps.get(0).getPosition2d(tempPoint2d);
         solver.resetFootstepRegularization(tempPoint2d);
      }
   }

   private boolean computeWhetherToIncludeFootsteps()
   {
      if (!localUseStepAdjustment || isInDoubleSupport.getBooleanValue() || isStanding.getBooleanValue())
         return false;

      return upcomingFootsteps.size() > 0;
   }

   @Override
   public boolean useStepAdjustment()
   {
      return useStepAdjustment.getBooleanValue();
   }

   @Override
   public void getDesiredCMP(FramePoint2D desiredCMP)
   {
      feedbackCMP.getFrameTuple2d(desiredCMP);
   }

   @Override
   public void getFootstepSolution(FramePoint2D footstepSolutionToPack)
   {
      footstepSolution.getFrameTuple2d(footstepSolutionToPack);
   }

   @Override
   public boolean wasFootstepAdjusted()
   {
      return solutionHandler.wasFootstepAdjusted();
   }

   @Override
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getBooleanValue();
   }

   @Override
   public void compute(double currentTime, FramePoint2D desiredICP, FrameVector2D desiredICPVelocity, FramePoint2D perfectCMP, FramePoint2D currentICP, double omega0)
   {
      controllerTimer.startMeasurement();

      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      this.currentICP.set(currentICP);
      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);
      this.yoPerfectCMP.set(perfectCMP);

      this.icpError.set(currentICP);
      this.icpError.sub(desiredICP);

      updateYoFootsteps();

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      boolean includeFootsteps = computeWhetherToIncludeFootsteps();

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(omega0, includeFootsteps);

      qpSolverTimer.startMeasurement();
      NoConvergenceException noConvergenceException = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(noConvergenceException, includeFootsteps);

      modifyAngularMomentumWeightUsingIntegral();

      controllerTimer.stopMeasurement();
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

   private void submitSolverTaskConditions(double omega0, boolean includeFootsteps)
   {
      if (isInDoubleSupport.getBooleanValue())
      {
         copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      }
      else
      {
         copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide.getEnumValue(), solver);
      }

      solver.resetFootstepConditions();

      if (localUseStepAdjustment && !isInDoubleSupport.getBooleanValue())
      {
         submitFootstepTaskConditionsToSolver(omega0, includeFootsteps);
         reachabilityConstraintHandler.updateReachabilityConstraint(solver);
      }
      else
      {
         predictedEndOfStateICP.setToNaN();
      }

      submitFeedbackTaskConditionsToSolver();
      submitAngularMomentumTaskConditionsToSolver();
   }

   private void submitFeedbackTaskConditionsToSolver()
   {
      helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);

      double dynamicsObjectiveWeight = this.dynamicsObjectiveWeight.getDoubleValue();
      if (isInDoubleSupport.getBooleanValue())
         dynamicsObjectiveWeight = dynamicsObjectiveWeight / dynamicsObjectiveDoubleSupportWeightModifier;

      solver.resetFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), tempVector2d.getX(), tempVector2d.getY(), dynamicsObjectiveWeight);
      solver.setMaxCMPDistanceFromEdge(maxAllowedDistanceCMPSupport.getDoubleValue());
      solver.setCopSafeDistanceToEdge(safeCoPDistanceToEdge.getDoubleValue());

      if (useFeedbackRegularization)
         solver.setFeedbackRegularizationWeight(feedbackRegularizationWeight.getDoubleValue() / controlDT);
   }

   private void submitAngularMomentumTaskConditionsToSolver()
   {
      double angularMomentumMinimizationWeight = this.scaledAngularMomentumMinimizationWeight.getDoubleValue();

      solver.resetAngularMomentumConditions();
      solver.setAngularMomentumConditions(angularMomentumMinimizationWeight, useAngularMomentum.getBooleanValue());
   }

   private void submitFootstepTaskConditionsToSolver(double omega0, boolean includeFootsteps)
   {
      if (includeFootsteps)
      {
         ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();
         helper.transformToWorldFrame(tempVector2d, forwardFootstepWeight, lateralFootstepWeight, soleFrame);
         footstepWeights.set(tempVector2d);

         double recursionTime = timeRemainingInState.getDoubleValue() + transferDurationSplitFraction.getDoubleValue() * nextTransferDuration.getDoubleValue();
         double recursionMultiplier = Math.exp(-omega0 * recursionTime);
         this.footstepMultiplier.set(recursionMultiplier);

         yoPerfectCMP.getFrameTuple2d(perfectCMP);
         predictedEndOfStateICP.set(desiredICP);
         predictedEndOfStateICP.sub(perfectCMP);
         predictedEndOfStateICP.scale(Math.exp(omega0 * timeRemainingInState.getDoubleValue()));
         predictedEndOfStateICP.add(perfectCMP);

         upcomingFootsteps.get(0).getPosition2d(tempPoint2d);
         solver.setFootstepAdjustmentConditions(recursionMultiplier, footstepWeights.getX(), footstepWeights.getY(), footstepAdjustmentSafetyFactor.getDoubleValue(), tempPoint2d);
      }

      if (useFootstepRegularization)
         solver.setFootstepRegularizationWeight(scaledFootstepRegularizationWeight.getDoubleValue() / controlDT);
   }

   private NoConvergenceException solveQP()
   {
      NoConvergenceException noConvergenceException = null;
      try
      {
         yoPerfectCMP.getFrameTuple2d(perfectCMP);
         solver.compute(icpError, perfectCMP);
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

   private void extractSolutionsFromSolver(NoConvergenceException noConvergenceException, boolean includeFootsteps)
   {
      // don't pole the new solutions if there's a no convergence exception
      if (noConvergenceException == null)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment && includeFootsteps)
            solutionHandler.extractFootstepSolutions(footstepSolution, unclippedFootstepSolution, upcomingFootsteps.get(0), solver);

         if (isInDoubleSupport.getBooleanValue())
            solutionHandler.zeroAdjustment();

         solutionHandler.updateVisualizers(desiredICP, footstepMultiplier.getDoubleValue());

         solver.getCoPFeedbackDifference(tempVector2d);
         feedbackCoPDelta.set(tempVector2d);

         solver.getCMPDifferenceFromCoP(tempVector2d);
         cmpCoPDifferenceSolution.set(tempVector2d);

         if (COMPUTE_COST_TO_GO)
            solutionHandler.updateCostsToGo(solver);
      }

      feedbackCMPDelta.set(feedbackCoPDelta);
      feedbackCMPDelta.add(cmpCoPDifferenceSolution);

      yoPerfectCMP.getFrameTuple2d(perfectCMP);
      feedbackCMP.set(perfectCMP);
      feedbackCMP.add(feedbackCMPDelta);

      if (limitReachabilityFromAdjustment.getBooleanValue() && localUseStepAdjustment && includeFootsteps)
         updateReachabilityRegionFromAdjustment();
   }

   private void updateReachabilityRegionFromAdjustment()
   {
      reachabilityConstraintHandler.updateReachabilityBasedOnAdjustment(upcomingFootsteps.get(0), unclippedFootstepSolution, wasFootstepAdjusted());
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
         if (isInDoubleSupport.getBooleanValue())
            timeRemainingInState.set(transferDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
         else
            timeRemainingInState.set(swingDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      }
   }

   private void scaleStepRegularizationWeightWithTime()
   {
      if (scaleStepRegularizationWeightWithTime.getBooleanValue())
      {
         double alpha = Math.max(timeRemainingInState.getDoubleValue(), minimumTimeRemaining.getDoubleValue()) / swingDuration.getDoubleValue();
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

      //helper.transformToWorldFrame(tempVector2d, feedbackForwardWeight, feedbackLateralWeight, soleFrame);
      helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackForwardWeight, feedbackLateralWeight);
      scaledFeedbackWeight.set(tempVector2d);

      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);
         scaledFeedbackWeight.scale(1.0 / tempVector2d.length());
      }
   }

   private void modifyAngularMomentumWeightUsingIntegral()
   {
      double angularMomentumMinimizationWeight = this.angularMomentumMinimizationWeight.getDoubleValue();

      if (!useAngularMomentumIntegrator.getBooleanValue())
      {
         scaledAngularMomentumMinimizationWeight.set(angularMomentumMinimizationWeight);
         return;
      }

      double angularMomentumMagnitude = cmpCoPDifferenceSolution.length();

      double cumulativeAngularMomentumAfterLeak = angularMomentumMagnitude * controlDT +
            angularMomentumIntegratorLeakRatio.getDoubleValue() * cumulativeAngularMomentum.getDoubleValue();
      cumulativeAngularMomentum.set(cumulativeAngularMomentumAfterLeak);

      double multiplier = 1.0 + angularMomentumIntegratorGain.getDoubleValue() * cumulativeAngularMomentumAfterLeak;

      scaledAngularMomentumMinimizationWeight.set(multiplier * angularMomentumMinimizationWeight);
   }
}
