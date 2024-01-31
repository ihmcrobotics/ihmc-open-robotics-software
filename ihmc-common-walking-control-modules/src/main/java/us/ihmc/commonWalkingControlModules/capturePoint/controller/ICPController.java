package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.scs2.definition.visual.ColorDefinitions.DarkOrange;
import static us.ihmc.scs2.definition.visual.ColorDefinitions.Purple;
import static us.ihmc.scs2.definition.visual.ColorDefinitions.Red;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_PLUS;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPController implements ICPControllerInterface
{
   private static final boolean VISUALIZE = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final BooleanProvider scaleFeedbackWeightWithGain;

   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   private final YoFrameVector2D expectedControlICPVelocity = new YoFrameVector2D(yoNamePrefix + "ExpectedControlICPVelocity", worldFrame, registry);

   private final YoFrameVector2D unconstrainedFeedback = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedback", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMP", worldFrame, registry);
   private final YoFrameVector2D unconstrainedFeedbackNoScaling = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedbackNoScaling", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMPNoScaling = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMPNoScaling", worldFrame, registry);
   final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   private final YoFramePoint2D referenceFeedForwardCoP = new YoFramePoint2D(yoNamePrefix + "ReferenceFeedForwardCoP", worldFrame, registry);
   private final YoFrameVector2D referenceFeedForwardCMPOffset = new YoFrameVector2D(yoNamePrefix + "ReferenceFeedForwardCMPOffset", worldFrame, registry);

   final YoFrameVector2D feedbackCoPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   final YoFrameVector2D feedbackCMPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);

   private final YoDouble feedbackAlpha = new YoDouble(yoNamePrefix + "FeedbackAlpha", registry);
   private final YoDouble feedForwardAlpha = new YoDouble(yoNamePrefix + "FeedForwardAlpha", registry);

   private final YoFrameVector2D residualDynamicsError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final DoubleProvider copFeedbackForwardWeight;
   private final DoubleProvider copFeedbackLateralWeight;
   private final DoubleProvider cmpFeedbackWeight;
   private final DMatrixRMaj scaledCoPFeedbackWeight = new DMatrixRMaj(2, 2);

   private final DoubleProvider maxAllowedDistanceCMPSupport;
   private final DoubleProvider safeCoPDistanceToEdge;

   private final DoubleProvider feedbackRateWeight;
   private final DoubleProvider copCMPFeedbackRateWeight;
   private final DoubleProvider dynamicsObjectiveWeight;

   private final DoubleProvider feedbackDirectionWeight;

   private final AngularMomentumIntegrator integrator;

   private final ICPControlGainsReadOnly feedbackGains;
   private final DMatrixRMaj transformedGains = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj inverseTransformedGains = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj transformedMagnitudeJacobian = new DMatrixRMaj(2, 2);

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoBoolean previousTickFailed = new YoBoolean(yoNamePrefix + "PreviousTickFailed", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final IntegerProvider maxNumberOfIterations = new IntegerParameter(yoNamePrefix + "MaxNumberOfIterations", registry, 100);

   private final ICPCoPConstraintHandler copConstraintHandler;
   private final ICPControllerQPSolver solver;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D finalICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();

   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();

   private final ICPControllerParameters parameters;

   private final double controlDT;
   private final double controlDTSquare;

   private final ICPControllerHelper helper = new ICPControllerHelper();

   public ICPController(WalkingControllerParameters walkingControllerParameters,
                        ICPControlPolygons icpControlPolygons,
                        SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                        double controlDT,
                        YoRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters,
           walkingControllerParameters.getICPControllerParameters(),
           icpControlPolygons,
           contactableFeet,
           controlDT,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public ICPController(WalkingControllerParameters walkingControllerParameters,
                        ICPControllerParameters icpOptimizationParameters,
                        ICPControlPolygons icpControlPolygons,
                        SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                        double controlDT,
                        YoRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.parameters = icpOptimizationParameters;
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

      feedbackDirectionWeight = new DoubleParameter(yoNamePrefix + "FeedbackDirectionWeight", registry, icpOptimizationParameters.getFeedbackDirectionWeight());

      BooleanProvider useICPControlPolygons = new BooleanParameter(yoNamePrefix + "UseICPControlPolygons",
                                                                   registry,
                                                                   icpOptimizationParameters.getUseICPControlPolygons());
      boolean hasICPControlPolygons = icpControlPolygons != null;

      safeCoPDistanceToEdge = new DoubleParameter(yoNamePrefix + "SafeCoPDistanceToEdge", registry, icpOptimizationParameters.getSafeCoPDistanceToEdge());
      double defaultMaxAllowedDistanceCMPSupport = walkingControllerParameters != null ? walkingControllerParameters.getMaxAllowedDistanceCMPSupport()
                                                                                       : Double.NaN;
      maxAllowedDistanceCMPSupport = new DoubleParameter(yoNamePrefix + "MaxAllowedDistanceCMPSupport", registry, defaultMaxAllowedDistanceCMPSupport);

      integrator = new AngularMomentumIntegrator(yoNamePrefix, icpOptimizationParameters, feedbackGains, controlDT, registry);

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRateAutomatically = true;
      solver = new ICPControllerQPSolver(totalVertices, updateRateAutomatically, registry);

      copConstraintHandler = new ICPCoPConstraintHandler(icpControlPolygons, useICPControlPolygons, hasICPControlPolygons, registry);

      parameters.createFeedForwardAlphaCalculator(registry, yoGraphicsListRegistry);
      parameters.createFeedbackAlphaCalculator(registry, null);
      parameters.createFeedbackProjectionOperator(registry, null);

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
      YoGraphicPosition feedForwardCoP = new YoGraphicPosition(yoNamePrefix + "ReferenceFeedForwardCoP",
                                                               this.referenceFeedForwardCoP,
                                                               0.005,
                                                               YoAppearance.Red(),
                                                               YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCMP = new YoGraphicPosition(yoNamePrefix + "UnconstrainedFeedbackCMP",
                                                                         this.unconstrainedFeedbackCMP,
                                                                         0.006,
                                                                         YoAppearance.Purple(),
                                                                         GraphicType.BALL_WITH_CROSS);

      artifactList.add(feedbackCoP.createArtifact());
      artifactList.add(feedForwardCoP.createArtifact());
      artifactList.add(unconstrainedFeedbackCMP.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void initialize()
   {
      integrator.reset();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint2DReadOnly getDesiredCMP()
   {
      return feedbackCMP;
   }

   /**
    * {@inheritDoc}
    * 
    * @return
    */
   @Override
   public FramePoint2DReadOnly getDesiredCoP()
   {
      return feedbackCoP;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector2DReadOnly getExpectedControlICPVelocity()
   {
      return expectedControlICPVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getValue();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                       FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly finalICP,
                       FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      controllerTimer.startMeasurement();

      this.desiredICP.setMatchingFrame(desiredICP);
      this.finalICP.setMatchingFrame(finalICP);
      this.desiredICPVelocity.setMatchingFrame(desiredICPVelocity);
      if (perfectCMPOffset == null)
         this.perfectCMPOffset.setToZero();
      else
         this.perfectCMPOffset.setMatchingFrame(perfectCMPOffset);
      this.currentICP.setMatchingFrame(currentICP);
      this.currentCoMPosition.setMatchingFrame(currentCoMPosition);

      CapturePointTools.computeCenterOfMassVelocity(currentCoMPosition, currentICP, omega0, currentCoMVelocity);

      if (perfectCoP == null)
      { // Then compute the perfect CMP using: x_{CMP} = x_{ICP} - xDot_{ICP} / omega0, and perfect CoP using the perfect CMP offset.
         this.perfectCMP.scaleAdd(-1.0 / omega0, desiredICPVelocity, desiredICP);
         this.perfectCoP.sub(this.perfectCMP, this.perfectCMPOffset);
      }
      else
      {
         this.perfectCoP.setMatchingFrame(perfectCoP);
         this.perfectCMP.add(this.perfectCoP, this.perfectCMPOffset);
      }

      this.icpError.sub(currentICP, desiredICP);

      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(supportPolygonInWorld);

      solver.setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      qpSolverTimer.startMeasurement();
      boolean converged = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(converged);

      computeCMPPositions();

      expectedControlICPVelocity.sub(currentICP, feedbackCMP);
      expectedControlICPVelocity.scale(omega0);

      controllerTimer.stopMeasurement();
   }

   private void submitSolverTaskConditions(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraint(supportPolygonInWorld));

      if (copConstraintHandler.hasSupportPolygonChanged())
         solver.notifyResetActiveSet();

      helper.transformGainsFromDynamicsFrame(transformedGains,
                                             desiredICPVelocity,
                                             feedbackGains.getKpParallelToMotion(),
                                             feedbackGains.getKpOrthogonalToMotion());
      helper.getTransformToDynamicsFrame(transformedMagnitudeJacobian, desiredICPVelocity);

      // run a temporary computation here to get what the  unconstrained CMP would be with none of the scaling
      computeUnconstrainedFeedbackCMP(perfectCoP, perfectCMPOffset, unconstrainedFeedbackNoScaling, unconstrainedFeedbackCMPNoScaling);
      computeFeedForwardAndFeedBackAlphas();

      referenceFeedForwardCMPOffset.setAndScale(1.0 - feedForwardAlpha.getDoubleValue(), perfectCMPOffset);
      referenceFeedForwardCoP.interpolate(perfectCoP, desiredICP, feedForwardAlpha.getDoubleValue());

      computeUnconstrainedFeedbackCMP(referenceFeedForwardCoP, referenceFeedForwardCMPOffset, unconstrainedFeedback, unconstrainedFeedbackCMP);

      UnrolledInverseFromMinor_DDRM.inv(transformedGains, inverseTransformedGains);

      solver.resetCoPFeedbackConditions();
      solver.resetFeedbackDirection();
      solver.setFeedbackConditions(scaledCoPFeedbackWeight, transformedGains, dynamicsObjectiveWeight.getValue());
      solver.setMaxCMPDistanceFromEdge(maxAllowedDistanceCMPSupport.getValue());
      solver.setCopSafeDistanceToEdge(safeCoPDistanceToEdge.getValue());
      solver.setDesiredFeedbackDirection(unconstrainedFeedback, feedbackDirectionWeight.getValue());

      if (ICPControllerHelper.isStationary(desiredICPVelocity))
      {
         solver.setMaximumFeedbackMagnitude(transformedMagnitudeJacobian,
                                            feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion(),
                                            feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion());
      }
      else
      {
         solver.setMaximumFeedbackMagnitude(transformedMagnitudeJacobian,
                                            feedbackGains.getFeedbackPartMaxValueParallelToMotion(),
                                            feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion());
      }
      solver.setMaximumFeedbackRate(feedbackGains.getFeedbackPartMaxRate(), controlDT);

      solver.setFeedbackRateWeight(copCMPFeedbackRateWeight.getValue() / controlDTSquare, feedbackRateWeight.getValue() / controlDTSquare);

      if (useCMPFeedback.getValue())
         solver.setCMPFeedbackConditions(cmpFeedbackWeight.getValue(), useAngularMomentum.getValue());
   }

   private boolean solveQP()
   {
      boolean converged = solver.compute(icpError, referenceFeedForwardCoP, referenceFeedForwardCMPOffset);
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

   private void computeUnconstrainedFeedbackCMP(FramePoint2DReadOnly feedforwardCoP,
                                                FrameVector2DReadOnly feedForwardCMPOffset,
                                                FixedFrameVector2DBasics unconstrainedFeedbackToPack,
                                                FixedFramePoint2DBasics unconstrainedFeedbackCMPToPack)
   {
      unconstrainedFeedbackToPack.setX(transformedGains.get(0, 0) * icpError.getX() + transformedGains.get(0, 1) * icpError.getY());
      unconstrainedFeedbackToPack.setY(transformedGains.get(1, 0) * icpError.getX() + transformedGains.get(1, 1) * icpError.getY());
      unconstrainedFeedbackCMPToPack.add(feedforwardCoP, feedForwardCMPOffset);
      unconstrainedFeedbackCMPToPack.add(unconstrainedFeedbackToPack);
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
      }

      integrator.update(desiredICPVelocity, currentCoMVelocity, icpError);
   }

   private void computeFeedForwardAndFeedBackAlphas()
   {
      if (parameters.getFeedbackAlphaCalculator() != null)
         feedbackAlpha.set(parameters.getFeedbackAlphaCalculator().computeAlpha(currentICP, copConstraintHandler.getCoPConstraint()));
      else
         feedbackAlpha.set(0.0);

      if (parameters.getFeedForwardAlphaCalculator() != null)
         feedForwardAlpha.set(parameters.getFeedForwardAlphaCalculator()
                                        .computeAlpha(currentICP,
                                                      desiredICP,
                                                      finalICP,
                                                      perfectCMP,
                                                      unconstrainedFeedbackCMPNoScaling,
                                                      copConstraintHandler.getCoPConstraint()));
      else
         feedForwardAlpha.set(0.0);

      // catches a few bugs
      if (feedForwardAlpha.isNaN())
         feedForwardAlpha.set(0.0);
   }

   private void computeCMPPositions()
   {
      feedbackCoP.set(referenceFeedForwardCoP);
      feedbackCoP.scaleAdd(1.0 - feedbackAlpha.getValue(), feedbackCoPDelta, feedbackCoP);
      feedbackCMP.add(referenceFeedForwardCMPOffset, feedbackCoP);
      feedbackCMP.scaleAdd(1.0 - feedbackAlpha.getValue(), feedbackCMPDelta, feedbackCMP);

      if (parameters.getFeedbackProjectionOperator() != null)
      {
         parameters.getFeedbackProjectionOperator()
                   .projectFeedback(currentICP, unconstrainedFeedbackCMP, perfectCMPOffset, copConstraintHandler.getCoPConstraint(), feedbackCoP, feedbackCMP);
      }

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
         CommonOps_DDRM.scale(1.0 / magnitude, scaledCoPFeedbackWeight);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.copConstraintHandler.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicPoint2D("FeedbackCoP", feedbackCoP, 0.01, DarkOrange(), CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("ReferenceFeedForwardCoP", referenceFeedForwardCoP, 0.01, Red(), CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("UnconstrainedFeedbackCMP", unconstrainedFeedbackCMP, 0.012, Purple(), CIRCLE_PLUS));
      group.setVisible(VISUALIZE);
      return group;

   }
}
