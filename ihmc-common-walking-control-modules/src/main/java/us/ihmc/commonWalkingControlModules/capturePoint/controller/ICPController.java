package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationCoPConstraintHandler;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class ICPController
{
   private static final boolean VISUALIZE = true;
   private static final boolean COMPUTE_COST_TO_GO = false;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final BooleanProvider scaleFeedbackWeightWithGain;

   private final YoBoolean isStationary = new YoBoolean(yoNamePrefix + "IsStationary", registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);
   private final YoDouble transferDuration = new YoDouble(yoNamePrefix + "TransferDuration", registry);

   private final YoEnum<RobotSide> supportSide = new YoEnum<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);
   private final DoubleProvider minimumTimeRemaining;

   private final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   private final YoFramePoint2D yoPerfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);
   private final YoFramePoint2D predictedEndOfStateICP = new YoFramePoint2D(yoNamePrefix + "PredictedEndOfStateICP", worldFrame, registry);
   private final YoDouble currentICPVelocityMagnitude = new YoDouble(yoNamePrefix + "ICPVelocityMagnitude", registry);

   private final YoFrameVector2D feedbackCoPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   private final YoFrameVector2D feedbackCMPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);

   private final YoFrameVector2D dynamicsError = new YoFrameVector2D(yoNamePrefix + "DynamicsError", worldFrame, registry);

   private final DoubleProvider copFeedbackForwardWeight;
   private final DoubleProvider copFeedbackLateralWeight;
   private final DoubleProvider cmpFeedbackWeight;
   private final YoMatrix yoScaledCoPFeedbackWeight = new YoMatrix(yoNamePrefix + "ScaledCoPFeedbackWeight", 2, 2, registry);
   private final YoDouble scaledCMPFeedbackWeight = new YoDouble(yoNamePrefix + "ScaledCMPFeedbackWeight", registry);
   private final DenseMatrix64F scaledCoPFeedbackWeight = new DenseMatrix64F(2, 2);

   private final DoubleProvider maxAllowedDistanceCMPSupport;
   private final DoubleProvider safeCoPDistanceToEdge;

   private final DoubleProvider feedbackRateWeight;
   private final DoubleProvider copCMPFeedbackRateWeight;
   private final DoubleProvider dynamicsObjectiveWeight;

   private final YoDouble cumulativeAngularMomentum = new YoDouble(yoNamePrefix + "CumulativeAngularMomentum", registry);

   private final BooleanProvider useICPControlPolygons;
   private final boolean hasICPControlPolygons;

   private final ICPControlGainsReadOnly feedbackGains;
   private final DenseMatrix64F transformedGains = new DenseMatrix64F(2, 2);
   private final FrameVector2D transformedMagnitudeLimits = new FrameVector2D();

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoBoolean previousTickFailed = new YoBoolean(yoNamePrefix + "PreviousTickFailed", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final IntegerProvider maxNumberOfIterations = new IntegerParameter(yoNamePrefix + "MaxNumberOfIterations", registry, 100);


   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final BooleanProvider useAngularMomentumIntegrator;
   private final DoubleProvider angularMomentumIntegratorGain;
   private final DoubleProvider angularMomentumIntegratorLeakRatio;

   private final BooleanProvider useSmartICPIntegrator;
   private final GlitchFilteredYoBoolean isICPStuck;
   private final DoubleProvider thresholdForStuck;
   private final YoFrameVector2D feedbackCMPIntegral = new YoFrameVector2D(yoNamePrefix + "FeedbackCMPIntegral", worldFrame, registry);

   private final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   private final ICPControllerQPSolver solver;

   private final ICPControlPlane icpControlPlane;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final BooleanProvider useFeedbackRate;

   private final FrameVector2D tempVector2d = new FrameVector2D();

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FrameVector2D currentICPVelocity = new FrameVector2D();

   private final double controlDT;
   private final double controlDTSquare;
   private final DoubleProvider dynamicsObjectiveDoubleSupportWeightModifier;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   private boolean initialized = false;

   public ICPController(WalkingControllerParameters walkingControllerParameters,
                        BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                        SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons,
           contactableFeet, controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public ICPController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                        BipedSupportPolygons bipedSupportPolygons,
                        ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                        YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;

      if (icpControlPolygons != null)
         this.icpControlPlane = icpControlPolygons.getIcpControlPlane();
      else
         this.icpControlPlane = null;
      hasICPControlPolygons = this.icpControlPlane != null;

      dynamicsObjectiveDoubleSupportWeightModifier = new DoubleParameter(yoNamePrefix + "DynamicsObjectiveDoubleSupportWeightModifier", registry,
                                                                         icpOptimizationParameters.getDynamicsObjectiveDoubleSupportWeightModifier());

      useFeedbackRate = new BooleanParameter(yoNamePrefix + "UseFeedbackRate", registry, icpOptimizationParameters.useFeedbackRate());

      useCMPFeedback = new BooleanParameter(yoNamePrefix + "UseCMPFeedback", registry, icpOptimizationParameters.useCMPFeedback());
      useAngularMomentum = new BooleanParameter(yoNamePrefix + "UseAngularMomentum", registry, icpOptimizationParameters.useAngularMomentum());

      scaleFeedbackWeightWithGain = new BooleanParameter(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry,
                                                         icpOptimizationParameters.scaleFeedbackWeightWithGain());

      copFeedbackForwardWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackForwardWeight", registry, icpOptimizationParameters.getFeedbackForwardWeight());
      copFeedbackLateralWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackLateralWeight", registry, icpOptimizationParameters.getFeedbackLateralWeight());

      copCMPFeedbackRateWeight = new DoubleParameter(yoNamePrefix + "CoPCMPFeedbackRateWeight", registry,
                                                     icpOptimizationParameters.getCoPCMPFeedbackRateWeight());
      feedbackRateWeight = new DoubleParameter(yoNamePrefix + "FeedbackRateWeight", registry, icpOptimizationParameters.getFeedbackRateWeight());

      feedbackGains = new ParameterizedICPControlGains("", icpOptimizationParameters.getICPFeedbackGains(), registry);
      useSmartICPIntegrator = new BooleanParameter("useSmartICPIntegrator", registry, icpOptimizationParameters.useSmartICPIntegrator());
      thresholdForStuck = new DoubleParameter(yoNamePrefix + "ThresholdForStuck", registry, icpOptimizationParameters.getICPVelocityThresholdForStuck());

      dynamicsObjectiveWeight = new DoubleParameter(yoNamePrefix + "DynamicsObjectiveWeight", registry, icpOptimizationParameters.getDynamicsObjectiveWeight());
      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      cmpFeedbackWeight = new DoubleParameter(yoNamePrefix + "CMPFeedbackWeight", registry, icpOptimizationParameters.getAngularMomentumMinimizationWeight());

      useAngularMomentumIntegrator = new BooleanParameter(yoNamePrefix + "UseAngularMomentumIntegrator", registry,
                                                          icpOptimizationParameters.getUseAngularMomentumIntegrator());
      angularMomentumIntegratorGain = new DoubleParameter(yoNamePrefix + "AngularMomentumIntegratorGain", registry,
                                                          icpOptimizationParameters.getAngularMomentumIntegratorGain());
      angularMomentumIntegratorLeakRatio = new DoubleParameter(yoNamePrefix + "AngularMomentumIntegratorLeakRatio", registry,
                                                               icpOptimizationParameters.getAngularMomentumIntegratorLeakRatio());

      useICPControlPolygons = new BooleanParameter(yoNamePrefix + "UseICPControlPolygons", registry, icpOptimizationParameters.getUseICPControlPolygons());

      safeCoPDistanceToEdge = new DoubleParameter(yoNamePrefix + "SafeCoPDistanceToEdge", registry, icpOptimizationParameters.getSafeCoPDistanceToEdge());
      double defaultMaxAllowedDistanceCMPSupport =
            walkingControllerParameters != null ? walkingControllerParameters.getMaxAllowedDistanceCMPSupport() : Double.NaN;
      maxAllowedDistanceCMPSupport = new DoubleParameter(yoNamePrefix + "MaxAllowedDistanceCMPSupport", registry, defaultMaxAllowedDistanceCMPSupport);

      isICPStuck = new GlitchFilteredYoBoolean(yoNamePrefix + "IsICPStuck", registry, (int) (0.03 / controlDT));

      minimumTimeRemaining = new DoubleParameter(yoNamePrefix + "MinimumTimeRemaining", registry, icpOptimizationParameters.getMinimumTimeRemaining());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRateAutomatically = true;
      solver = new ICPControllerQPSolver(totalVertices, COMPUTE_COST_TO_GO, updateRateAutomatically, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons, useICPControlPolygons, hasICPControlPolygons,
                                                                     registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition predictedEndOfStateICP = new YoGraphicPosition(yoNamePrefix + "PredictedEndOfStateICP", this.predictedEndOfStateICP, 0.005,
                                                                       YoAppearance.MidnightBlue(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition feedbackCoP = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(),
                                                            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      artifactList.add(predictedEndOfStateICP.createArtifact());
      artifactList.add(feedbackCoP.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /** {@inheritDoc} */
   public void clearPlan()
   {
      transferDuration.setToNaN();
      swingDuration.setToNaN();
   }

   /** {@inheritDoc} */
   public void setTransferDuration(double duration)
   {
      transferDuration.set(duration);
   }

   /** {@inheritDoc} */
   public void setSwingDuration(double duration)
   {
      swingDuration.set(duration);
   }

   /** {@inheritDoc} */
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStationary.set(true);
      isInDoubleSupport.set(true);
      isICPStuck.set(false);

      solver.resetCoPLocationConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());

      solver.notifyResetActiveSet();

      speedUpTime.set(0.0);
   }

   /** {@inheritDoc} */
   public void initializeForTransfer(double initialTime)
   {
      isInDoubleSupport.set(true);
      isStationary.set(false);
      isICPStuck.set(false);

      initializeOnContactChange(initialTime);

      solver.resetCoPLocationConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());

      solver.notifyResetActiveSet();
   }

   /** {@inheritDoc} */
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.supportSide.set(supportSide);
      isStationary.set(false);
      isInDoubleSupport.set(false);
      isICPStuck.set(false);

      initializeOnContactChange(initialTime);

      solver.resetCoPLocationConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide));

      solver.notifyResetActiveSet();
   }

   private void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      this.initialTime.set(initialTime);
   }

   /** {@inheritDoc} */
   public void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack)
   {
      desiredCMPToPack.set(feedbackCMP);
   }

   /** {@inheritDoc} */
   public void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack)
   {
      desiredCoPToPack.set(feedbackCoP);
   }


   /** {@inheritDoc} */
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getValue();
   }

   private final FrameVector2D desiredCMPOffsetToThrowAway = new FrameVector2D();

   /** {@inheritDoc} */
   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity, FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(currentTime, desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentICPVelocity, omega0);
   }

   /** {@inheritDoc} */
   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity, FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset, FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0)
   {
      controllerTimer.startMeasurement();

      if (!initialized)
      {
         initialize();
         initialized = true;
      }

      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);
      this.perfectCMPOffset.set(perfectCMPOffset);
      this.currentICP.set(currentICP);
      this.currentICPVelocity.set(currentICPVelocity);

      this.desiredICP.changeFrame(worldFrame);
      this.desiredICPVelocity.changeFrame(worldFrame);
      this.perfectCMPOffset.changeFrame(worldFrame);
      this.currentICP.changeFrame(worldFrame);
      this.currentICPVelocity.changeFrame(worldFrame);

      this.yoPerfectCoP.setMatchingFrame(perfectCoP);
      this.yoPerfectCMP.add(yoPerfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions();

      solver.setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      qpSolverTimer.startMeasurement();
      boolean converged = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(converged);

      modifyCMPFeedbackWeightUsingIntegral();

      controllerTimer.stopMeasurement();
   }

   private void initialize()
   {
      scaledCMPFeedbackWeight.set(cmpFeedbackWeight.getValue());
   }

   /** {@inheritDoc} */
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      if (swingSpeedUpEnabled.getBooleanValue() && remainingTimeForSwing < timeRemainingInState.getDoubleValue())
      {
         double speedUpTime = timeRemainingInState.getDoubleValue() - remainingTimeForSwing;
         this.speedUpTime.add(speedUpTime);
      }
   }

   private void submitSolverTaskConditions()
   {
         solver.resetCoPLocationConstraint();
         solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());

         if (copConstraintHandler.hasSupportPolygonChanged())
            solver.notifyResetActiveSet();


         predictedEndOfStateICP.setToNaN();

      submitCoPFeedbackTaskConditionsToSolver();
      if (useCMPFeedback.getValue())
         submitCMPFeedbackTaskConditionsToSolver();
   }

   private void submitCoPFeedbackTaskConditionsToSolver()
   {
      helper.transformGainsFromDynamicsFrame(transformedGains, desiredICPVelocity, feedbackGains.getKpParallelToMotion(),
                                             feedbackGains.getKpOrthogonalToMotion());
      helper.transformFromDynamicsFrame(transformedMagnitudeLimits, desiredICPVelocity, feedbackGains.getFeedbackPartMaxValueParallelToMotion(),
                                        feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion());

      double dynamicsObjectiveWeight = this.dynamicsObjectiveWeight.getValue();
      if (isInDoubleSupport.getBooleanValue())
         dynamicsObjectiveWeight = dynamicsObjectiveWeight / dynamicsObjectiveDoubleSupportWeightModifier.getValue();

      yoScaledCoPFeedbackWeight.get(scaledCoPFeedbackWeight);
      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledCoPFeedbackWeight, transformedGains, dynamicsObjectiveWeight);
      solver.setMaxCMPDistanceFromEdge(maxAllowedDistanceCMPSupport.getValue());
      solver.setCopSafeDistanceToEdge(safeCoPDistanceToEdge.getValue());

      solver.setMaximumFeedbackMagnitude(transformedMagnitudeLimits);
      solver.setMaximumFeedbackRate(feedbackGains.getFeedbackPartMaxRate(), controlDT);

      if (useFeedbackRate.getValue())
         solver.setFeedbackRateWeight(copCMPFeedbackRateWeight.getValue() / controlDTSquare, feedbackRateWeight.getValue() / controlDTSquare);
   }

   private void submitCMPFeedbackTaskConditionsToSolver()
   {
      double cmpFeedbackWeight = this.scaledCMPFeedbackWeight.getDoubleValue();

      solver.resetCMPFeedbackConditions();
      solver.setCMPFeedbackConditions(cmpFeedbackWeight, useAngularMomentum.getValue());
   }


   private boolean solveQP()
   {
      boolean converged = solver.compute(icpError, yoPerfectCoP, perfectCMPOffset);
      previousTickFailed.set(solver.previousTickFailed());
      if (!converged)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            LogTools.warn("The QP has not converged. Only showing this once if it happens repeatedly.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();
      }

      return converged;
   }

   private void extractSolutionsFromSolver(boolean converged)
   {
      numberOfIterations.set(solver.getNumberOfIterations());

      // Don't pole the new solutions if the solver has not converged.
      if (converged)
      {
         solver.getCoPFeedbackDifference(feedbackCoPDelta);
         solver.getCMPFeedbackDifference(feedbackCMPDelta);
         solver.getDynamicsError(dynamicsError);
      }

      isICPStuck.update(computeIsStuck());
      computeICPIntegralTerm();

      feedbackCoP.add(yoPerfectCoP, feedbackCoPDelta);
      feedbackCMP.add(feedbackCoP, perfectCMPOffset);
      feedbackCMP.add(feedbackCMPDelta);
      feedbackCMP.add(feedbackCMPIntegral);

   }


   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
   }

   private void computeTimeRemainingInState()
   {
      if (isStationary.getBooleanValue())
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

   private void scaleFeedbackWeightWithGain()
   {
      helper.transformFromDynamicsFrame(scaledCoPFeedbackWeight, desiredICPVelocity, copFeedbackForwardWeight.getValue(), copFeedbackLateralWeight.getValue());
      yoScaledCoPFeedbackWeight.set(scaledCoPFeedbackWeight);

      if (scaleFeedbackWeightWithGain.getValue())
      {
         double parallel = feedbackGains.getKpParallelToMotion();
         double orthogonal = feedbackGains.getKpOrthogonalToMotion();
         double magnitude = helper.transformGainsFromDynamicsFrame(transformedGains, desiredICPVelocity, parallel, orthogonal);
         CommonOps.scale(1.0 / magnitude, scaledCoPFeedbackWeight);
      }

      yoScaledCoPFeedbackWeight.set(scaledCoPFeedbackWeight);

   }

   private void modifyCMPFeedbackWeightUsingIntegral()
   {
      double cmpFeedbackWeight = this.cmpFeedbackWeight.getValue();

      if (!useAngularMomentumIntegrator.getValue())
      {
         scaledCMPFeedbackWeight.set(cmpFeedbackWeight);
         return;
      }

      double angularMomentumFeedbackMagnitude = feedbackCMPDelta.length() - perfectCMPOffset.length();

      double cumulativeAngularMomentumAfterLeak =
            angularMomentumFeedbackMagnitude * controlDT + angularMomentumIntegratorLeakRatio.getValue() * cumulativeAngularMomentum.getDoubleValue();
      cumulativeAngularMomentum.set(cumulativeAngularMomentumAfterLeak);

      double multiplier = 1.0 + angularMomentumIntegratorGain.getValue() * cumulativeAngularMomentumAfterLeak;

      scaledCMPFeedbackWeight.set(multiplier * cmpFeedbackWeight);
   }

   private boolean computeIsStuck()
   {
      if (!isInDoubleSupport.getBooleanValue() || isStationary.getBooleanValue())
         return false;

      if (isICPStuck.getBooleanValue())
         return true;

      currentICPVelocityMagnitude.set(currentICPVelocity.length());
      if ((currentICPVelocityMagnitude.getDoubleValue() < thresholdForStuck.getValue()) && (timeRemainingInState.getDoubleValue() <= minimumTimeRemaining.getValue()))
         return true;

      return false;
   }

   private void computeICPIntegralTerm()
   {
      if (useSmartICPIntegrator.getValue() && isICPStuck.getBooleanValue())
      {
         tempVector2d.set(icpError);
         tempVector2d.scale(controlDT * feedbackGains.getKi());

         feedbackCMPIntegral.scale(Math.pow(feedbackGains.getIntegralLeakRatio(), controlDT));
         feedbackCMPIntegral.add(tempVector2d);

         double length = feedbackCMPIntegral.length();
         double maxLength = feedbackGains.getMaxIntegralError();
         if (length > maxLength)
            feedbackCMPIntegral.scale(maxLength / length);
         if (Math.abs(feedbackGains.getKi()) < 1e-10)
            feedbackCMPIntegral.setToZero();
      }
      else
      {
         feedbackCMPIntegral.setToZero();
      }
   }

   /** {@inheritDoc} */
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.copConstraintHandler.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }
}
