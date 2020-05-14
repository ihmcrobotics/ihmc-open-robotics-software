package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
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

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final BooleanProvider scaleFeedbackWeightWithGain;

   private final YoBoolean isStationary = new YoBoolean(yoNamePrefix + "IsStationary", registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   private final YoEnum<RobotSide> supportSide = new YoEnum<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   final YoFrameVector2D feedbackCoPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   final YoFrameVector2D feedbackCMPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);
   private final DenseMatrix64F feedbackCMPDeltaMatrix = new DenseMatrix64F(2, 1);

   private final YoFrameVector2D residualDynamicsError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);
   private final YoFrameVector2D residualDynamicsErrorConservative = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsErrorConservative", worldFrame, registry);
   private final DenseMatrix64F residualDynamicsErrorConservativeMatrix = new DenseMatrix64F(2, 1);

   private final DoubleProvider copFeedbackForwardWeight;
   private final DoubleProvider copFeedbackLateralWeight;
   private final DoubleProvider cmpFeedbackWeight;
   private final DenseMatrix64F scaledCoPFeedbackWeight = new DenseMatrix64F(2, 2);

   private final DoubleProvider maxAllowedDistanceCMPSupport;
   private final DoubleProvider safeCoPDistanceToEdge;

   private final DoubleProvider feedbackRateWeight;
   private final DoubleProvider copCMPFeedbackRateWeight;
   private final DoubleProvider dynamicsObjectiveWeight;

   private final AngularMomentumIntegrator integrator;

   private final ICPControlGainsReadOnly feedbackGains;
   private final DenseMatrix64F transformedGains = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F inverseTransformedGains = new DenseMatrix64F(2, 2);
   private final FrameVector2D transformedMagnitudeLimits = new FrameVector2D();

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoBoolean previousTickFailed = new YoBoolean(yoNamePrefix + "PreviousTickFailed", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final IntegerProvider maxNumberOfIterations = new IntegerParameter(yoNamePrefix + "MaxNumberOfIterations", registry, 100);

   private final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   private final ICPControllerQPSolver solver;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FrameVector2D currentICPVelocity = new FrameVector2D();

   private final double controlDT;
   private final double controlDTSquare;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   public ICPController(WalkingControllerParameters walkingControllerParameters,
                        BipedSupportPolygons bipedSupportPolygons,
                        ICPControlPolygons icpControlPolygons,
                        SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                        double controlDT,
                        YoVariableRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters,
           walkingControllerParameters.getICPOptimizationParameters(),
           bipedSupportPolygons,
           icpControlPolygons,
           contactableFeet,
           controlDT,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public ICPController(WalkingControllerParameters walkingControllerParameters,
                        ICPOptimizationParameters icpOptimizationParameters,
                        BipedSupportPolygons bipedSupportPolygons,
                        ICPControlPolygons icpControlPolygons,
                        SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                        double controlDT,
                        YoVariableRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;


      useCMPFeedback = new BooleanParameter(yoNamePrefix + "UseCMPFeedback", registry, icpOptimizationParameters.useCMPFeedback());
      useAngularMomentum = new BooleanParameter(yoNamePrefix + "UseAngularMomentum", registry, icpOptimizationParameters.useAngularMomentum());

      scaleFeedbackWeightWithGain = new BooleanParameter(yoNamePrefix + "ScaleFeedbackWeightWithGain",
                                                         registry,
                                                         icpOptimizationParameters.scaleFeedbackWeightWithGain());

      copFeedbackForwardWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackForwardWeight", registry, icpOptimizationParameters.getFeedbackForwardWeight());
      copFeedbackLateralWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackLateralWeight", registry, icpOptimizationParameters.getFeedbackLateralWeight());

      copCMPFeedbackRateWeight = new DoubleParameter(yoNamePrefix + "CoPCMPFeedbackRateWeight",
                                                     registry,
                                                     icpOptimizationParameters.getCoPCMPFeedbackRateWeight());
      feedbackRateWeight = new DoubleParameter(yoNamePrefix + "FeedbackRateWeight", registry, icpOptimizationParameters.getFeedbackRateWeight());

      feedbackGains = new ParameterizedICPControlGains("", icpOptimizationParameters.getICPFeedbackGains(), registry);

      dynamicsObjectiveWeight = new DoubleParameter(yoNamePrefix + "DynamicsObjectiveWeight", registry, icpOptimizationParameters.getDynamicsObjectiveWeight());

      cmpFeedbackWeight = new DoubleParameter(yoNamePrefix + "CMPFeedbackWeight", registry, icpOptimizationParameters.getAngularMomentumMinimizationWeight());

      BooleanProvider useICPControlPolygons = new BooleanParameter(yoNamePrefix + "UseICPControlPolygons", registry, icpOptimizationParameters.getUseICPControlPolygons());
      boolean hasICPControlPolygons = icpControlPolygons != null;

      safeCoPDistanceToEdge = new DoubleParameter(yoNamePrefix + "SafeCoPDistanceToEdge", registry, icpOptimizationParameters.getSafeCoPDistanceToEdge());
      double defaultMaxAllowedDistanceCMPSupport =
            walkingControllerParameters != null ? walkingControllerParameters.getMaxAllowedDistanceCMPSupport() : Double.NaN;
      maxAllowedDistanceCMPSupport = new DoubleParameter(yoNamePrefix + "MaxAllowedDistanceCMPSupport", registry, defaultMaxAllowedDistanceCMPSupport);

      integrator = new AngularMomentumIntegrator(yoNamePrefix, icpOptimizationParameters, feedbackGains, controlDT, registry);

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRateAutomatically = true;
      solver = new ICPControllerQPSolver(totalVertices, updateRateAutomatically, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons,
                                                                     icpControlPolygons,
                                                                     useICPControlPolygons,
                                                                     hasICPControlPolygons,
                                                                     registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition feedbackCoP = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP",
                                                            this.feedbackCoP,
                                                            0.005,
                                                            YoAppearance.Darkorange(),
                                                            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      artifactList.add(feedbackCoP.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /** {@inheritDoc} */
   public void initializeForStanding()
   {
      isStationary.set(true);
      isInDoubleSupport.set(true);
      integrator.reset();
   }

   /** {@inheritDoc} */
   public void initializeForTransfer()
   {
      isInDoubleSupport.set(true);
      isStationary.set(false);
      integrator.reset();
   }

   /** {@inheritDoc} */
   public void initializeForSingleSupport(RobotSide supportSide)
   {
      this.supportSide.set(supportSide);
      isStationary.set(false);
      isInDoubleSupport.set(false);
      integrator.reset();
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
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP,
                       FrameVector2DReadOnly currentICPVelocity)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentICPVelocity);
   }

   /** {@inheritDoc} */
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset,
                       FramePoint2DReadOnly currentICP,
                       FrameVector2DReadOnly currentICPVelocity)
   {
      controllerTimer.startMeasurement();

      this.desiredICP.setMatchingFrame(desiredICP);
      this.desiredICPVelocity.setMatchingFrame(desiredICPVelocity);
      this.perfectCMPOffset.setMatchingFrame(perfectCMPOffset);
      this.currentICP.setMatchingFrame(currentICP);
      this.currentICPVelocity.setMatchingFrame(currentICPVelocity);

      this.perfectCoP.setMatchingFrame(perfectCoP);
      this.perfectCMP.add(this.perfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);

      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions();

      solver.setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      qpSolverTimer.startMeasurement();
      boolean converged = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(converged);

      controllerTimer.stopMeasurement();
   }

   private void submitSolverTaskConditions()
   {
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraint());

      if (copConstraintHandler.hasSupportPolygonChanged())
         solver.notifyResetActiveSet();

      helper.transformGainsFromDynamicsFrame(transformedGains,
                                             desiredICPVelocity,
                                             feedbackGains.getKpParallelToMotion(),
                                             feedbackGains.getKpOrthogonalToMotion());
      helper.transformFromDynamicsFrame(transformedMagnitudeLimits,
                                        desiredICPVelocity,
                                        feedbackGains.getFeedbackPartMaxValueParallelToMotion(),
                                        feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion());

      fastStaticInverse(transformedGains, inverseTransformedGains);

      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledCoPFeedbackWeight, transformedGains, dynamicsObjectiveWeight.getValue());
      solver.setMaxCMPDistanceFromEdge(maxAllowedDistanceCMPSupport.getValue());
      solver.setCopSafeDistanceToEdge(safeCoPDistanceToEdge.getValue());

      solver.setMaximumFeedbackMagnitude(transformedMagnitudeLimits);
      solver.setMaximumFeedbackRate(feedbackGains.getFeedbackPartMaxRate(), controlDT);

      solver.setFeedbackRateWeight(copCMPFeedbackRateWeight.getValue() / controlDTSquare, feedbackRateWeight.getValue() / controlDTSquare);

      if (useCMPFeedback.getValue())
         solver.setCMPFeedbackConditions(cmpFeedbackWeight.getValue(), useAngularMomentum.getValue());
   }

   private static void fastStaticInverse(DenseMatrix64F matrixToInvert, DenseMatrix64F invertedMatrixToPack)
   {
      double determinate = CommonOps.det(matrixToInvert);
      invertedMatrixToPack.set(0, 0,  matrixToInvert.get(1, 1) / determinate);
      invertedMatrixToPack.set(1, 1, matrixToInvert.get(0, 0) / determinate);
      invertedMatrixToPack.set(0, 1, -matrixToInvert.get(0, 1) / determinate);
      invertedMatrixToPack.set(1, 0, -matrixToInvert.get(1, 0) / determinate);
   }

   private boolean solveQP()
   {
      boolean converged = solver.compute(icpError, perfectCoP, perfectCMPOffset);
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

      // Don't poll the new solutions if the solver has not converged.
      if (converged)
      {
         solver.getCoPFeedbackDifference(feedbackCoPDelta);
         solver.getCMPFeedbackDifference(feedbackCMPDelta);
         solver.getResidualDynamicsError(residualDynamicsError);

         feedbackCoPDelta.get(feedbackCMPDeltaMatrix);
         CommonOps.mult(-1.0, inverseTransformedGains, feedbackCMPDeltaMatrix, residualDynamicsErrorConservativeMatrix);
         residualDynamicsErrorConservative.set(residualDynamicsErrorConservativeMatrix);
         residualDynamicsErrorConservative.add(icpError);
      }

      boolean checkIfStuck = !isInDoubleSupport.getBooleanValue() || isStationary.getBooleanValue();
      integrator.update(checkIfStuck, desiredICPVelocity, currentICPVelocity, icpError);

      feedbackCoP.add(perfectCoP, feedbackCoPDelta);
      feedbackCMP.add(feedbackCoP, perfectCMPOffset);
      feedbackCMP.add(feedbackCMPDelta);
      feedbackCMP.add(integrator.getFeedbackCMPIntegral());
   }

   private void scaleFeedbackWeightWithGain()
   {
      helper.transformFromDynamicsFrame(scaledCoPFeedbackWeight, desiredICPVelocity, copFeedbackForwardWeight.getValue(), copFeedbackLateralWeight.getValue());

      if (scaleFeedbackWeightWithGain.getValue())
      {
         double parallel = feedbackGains.getKpParallelToMotion();
         double orthogonal = feedbackGains.getKpOrthogonalToMotion();
         double magnitude = helper.transformGainsFromDynamicsFrame(transformedGains, desiredICPVelocity, parallel, orthogonal);
         CommonOps.scale(1.0 / magnitude, scaledCoPFeedbackWeight);
      }
   }

   /** {@inheritDoc} */
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.copConstraintHandler.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }

   public FrameVector2DReadOnly getResidualError()
   {
      return residualDynamicsErrorConservative;
   }
}
