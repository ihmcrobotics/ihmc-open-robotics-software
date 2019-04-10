package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.PlanarRegion;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationController implements ICPOptimizationControllerInterface
{
   private static final boolean VISUALIZE = true;
   private static final boolean DEBUG = true;
   private static final boolean COMPUTE_COST_TO_GO = false;

   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanProvider allowStepAdjustment;
   private final YoBoolean includeFootsteps = new YoBoolean(yoNamePrefix + "IncludeFootsteps", registry);
   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean footstepIsAdjustable = new YoBoolean(yoNamePrefix + "FootstepIsAdjustable", registry);

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final BooleanProvider scaleStepRateWeightWithTime;
   private final BooleanProvider scaleFeedbackWeightWithGain;

   private final YoBoolean isStationary = new YoBoolean(yoNamePrefix + "IsStationary", registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);
   private final YoDouble transferDuration = new YoDouble(yoNamePrefix + "TransferDuration", registry);
   private final YoDouble nextTransferDuration = new YoDouble(yoNamePrefix + "NextTransferDuration", registry);
   private final YoDouble finalTransferDuration = new YoDouble(yoNamePrefix + "FinalTransferDuration", registry);

   private final DoubleProvider transferDurationSplitFraction;

   private final YoEnum<RobotSide> transferToSide = new YoEnum<>(yoNamePrefix + "TransferToSide", registry, RobotSide.class, true);
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

   private final YoFrameVector2D feedbackCoPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   private final YoFrameVector2D feedbackCMPDelta = new YoFrameVector2D(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);

   private final YoFrameVector2D dynamicsError = new YoFrameVector2D(yoNamePrefix + "DynamicsError", worldFrame, registry);

   private final YoInteger numberOfRegisteredSteps = new YoInteger(yoNamePrefix + "NumberOfRegisteredSteps", registry);
   private final YoFramePose3D upcomingFootstep = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepPose", worldFrame, registry);
   private final YoEnum<RobotSide> upcomingFootstepSide = new YoEnum<>(yoNamePrefix + "UpcomingFootstepSide", registry, RobotSide.class);
   private final RecyclingArrayList<Point2D> upcomingFootstepContactPoints = new RecyclingArrayList<>(Point2D.class);

   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D footstepLocationSubmitted = new YoFramePoint2D(yoNamePrefix + "FootstepLocationSubmitted", worldFrame, registry);
   private final YoFramePoint2D unclippedFootstepSolution = new YoFramePoint2D(yoNamePrefix + "UnclippedFootstepSolutionLocation", worldFrame, registry);

   private final DoubleProvider minICPErrorForStepAdjustment;
   private final DoubleProvider fractionThroughSwingForAdjustment;

   private final DoubleProvider footstepAdjustmentSafetyFactor;
   private final DoubleProvider forwardFootstepWeight;
   private final DoubleProvider lateralFootstepWeight;
   private final YoMatrix yoFootstepWeights = new YoMatrix(yoNamePrefix + "FootstepWeights", 2, 2, registry);
   private final DenseMatrix64F footstepWeights = new DenseMatrix64F(2, 2);

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
   private final DoubleProvider footstepRateWeight;
   private final YoDouble scaledFootstepRateWeight = new YoDouble(yoNamePrefix + "ScaledFootstepRateWeight", registry);
   private final DoubleProvider dynamicsObjectiveWeight;

   private final YoDouble cumulativeAngularMomentum = new YoDouble(yoNamePrefix + "CumulativeAngularMomentum", registry);

   private final BooleanProvider limitReachabilityFromAdjustment;

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

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "TotalFootstepMultiplier", registry);
   private final YoDouble recursionTime = new YoDouble(yoNamePrefix + "RecursionTime", registry);
   private final YoDouble recursionMultiplier = new YoDouble(yoNamePrefix + "RecursionMultiplier", registry);

   private final DoubleProvider minimumFootstepMultiplier;
   private final DoubleProvider maximumTimeFromTransfer;

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
   private final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   private final PlanarRegionConstraintProvider planarRegionConstraintProvider;
   private final ICPOptimizationSolutionHandler solutionHandler;
   private final ICPOptimizationQPSolver solver;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final ICPControlPlane icpControlPlane;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final BooleanProvider useFootstepRate;
   private final BooleanProvider useFeedbackRate;

   private boolean localUseStepAdjustment;

   private final FramePoint3D projectedTempPoint3d = new FramePoint3D();
   private final FrameVector2D tempVector2d = new FrameVector2D();

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FramePoint2D perfectCoP = new FramePoint2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FrameVector2D currentICPVelocity = new FrameVector2D();

   private final double controlDT;
   private final double controlDTSquare;
   private final DoubleProvider dynamicsObjectiveDoubleSupportWeightModifier;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   private boolean initialized = false;

   private final BooleanProvider considerAngularMomentumInAdjustment;
   private final BooleanProvider considerFeedbackInAdjustment;

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, SideDependentList<ReferenceFrame> soleZUpFrames,
                                    BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), soleZUpFrames, bipedSupportPolygons, icpControlPolygons,
           contactableFeet, controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                    SideDependentList<ReferenceFrame> soleZUpFrames, BipedSupportPolygons bipedSupportPolygons,
                                    ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;
      this.contactableFeet = contactableFeet;

      if (icpControlPolygons != null)
         this.icpControlPlane = icpControlPolygons.getIcpControlPlane();
      else
         this.icpControlPlane = null;
      hasICPControlPolygons = this.icpControlPlane != null;

      dynamicsObjectiveDoubleSupportWeightModifier = new DoubleParameter(yoNamePrefix + "DynamicsObjectiveDoubleSupportWeightModifier", registry,
                                                                         icpOptimizationParameters.getDynamicsObjectiveDoubleSupportWeightModifier());

      useFootstepRate = new BooleanParameter(yoNamePrefix + "UseFootstepRate", registry, icpOptimizationParameters.useFootstepRate());
      useFeedbackRate = new BooleanParameter(yoNamePrefix + "UseFeedbackRate", registry, icpOptimizationParameters.useFeedbackRate());

      allowStepAdjustment = new BooleanParameter(yoNamePrefix + "AllowStepAdjustment", registry, icpOptimizationParameters.allowStepAdjustment());
      useCMPFeedback = new BooleanParameter(yoNamePrefix + "UseCMPFeedback", registry, icpOptimizationParameters.useCMPFeedback());
      useAngularMomentum = new BooleanParameter(yoNamePrefix + "UseAngularMomentum", registry, icpOptimizationParameters.useAngularMomentum());

      scaleStepRateWeightWithTime = new BooleanParameter(yoNamePrefix + "ScaleStepRateWeightWithTime", registry,
                                                         icpOptimizationParameters.scaleStepRateWeightWithTime());
      scaleFeedbackWeightWithGain = new BooleanParameter(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry,
                                                         icpOptimizationParameters.scaleFeedbackWeightWithGain());

      minICPErrorForStepAdjustment = new DoubleParameter(yoNamePrefix + "MinICPErrorForStepAdjustment", registry,
                                                         icpOptimizationParameters.getMinICPErrorForStepAdjustment());
      fractionThroughSwingForAdjustment = new DoubleParameter(yoNamePrefix + "FractionThroughSwingForAdjustment", registry,
                                                              icpOptimizationParameters.getFractionThroughSwingForAdjustment());

      footstepAdjustmentSafetyFactor = new DoubleParameter(yoNamePrefix + "FootstepAdjustmentSafetyFactor", registry,
                                                           icpOptimizationParameters.getFootstepAdjustmentSafetyFactor());
      forwardFootstepWeight = new DoubleParameter(yoNamePrefix + "ForwardFootstepWeight", registry, icpOptimizationParameters.getForwardFootstepWeight());
      lateralFootstepWeight = new DoubleParameter(yoNamePrefix + "LateralFootstepWeight", registry, icpOptimizationParameters.getLateralFootstepWeight());
      footstepRateWeight = new DoubleParameter(yoNamePrefix + "FootstepRateWeight", registry, icpOptimizationParameters.getFootstepRateWeight());

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

      limitReachabilityFromAdjustment = new BooleanParameter(yoNamePrefix + "LimitReachabilityFromAdjustment", registry,
                                                             icpOptimizationParameters.getLimitReachabilityFromAdjustment());

      transferDurationSplitFraction = new DoubleParameter(yoNamePrefix + "TransferDurationSplitFraction", registry,
                                                          icpOptimizationParameters.getTransferSplitFraction());

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
      minimumFootstepMultiplier = new DoubleParameter(yoNamePrefix + "MinimumFootstepMultiplier", registry,
                                                      icpOptimizationParameters.getMinimumFootstepMultiplier());
      maximumTimeFromTransfer = new DoubleParameter(yoNamePrefix + "MaximumTimeFromTransfer", registry,
                                                    icpOptimizationParameters.maximumTimeFromTransferInFootstepMultiplier());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRateAutomatically = true;
      solver = new ICPOptimizationQPSolver(totalVertices, COMPUTE_COST_TO_GO, updateRateAutomatically, registry);

      considerAngularMomentumInAdjustment = new BooleanParameter(yoNamePrefix + "ConsiderAngularMomentumInAdjustment", registry,
                                                                 icpOptimizationParameters.considerAngularMomentumInAdjustment());
      considerFeedbackInAdjustment = new BooleanParameter(yoNamePrefix + "ConsiderFeedbackInAdjustment", registry,
                                                          icpOptimizationParameters.considerFeedbackInAdjustment());

      solutionHandler = new ICPOptimizationSolutionHandler(icpControlPlane, icpOptimizationParameters, useICPControlPolygons, DEBUG, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons, useICPControlPolygons, hasICPControlPolygons,
                                                                     registry);
      if (walkingControllerParameters != null)
      {
         reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(soleZUpFrames, icpOptimizationParameters,
                                                                                          walkingControllerParameters.getSteppingParameters(), yoNamePrefix,
                                                                                          VISUALIZE, registry, yoGraphicsListRegistry);
         planarRegionConstraintProvider = new PlanarRegionConstraintProvider(icpControlPlane, walkingControllerParameters, icpOptimizationParameters,
                                                                             bipedSupportPolygons, soleZUpFrames, contactableFeet, yoNamePrefix, VISUALIZE,
                                                                             registry, yoGraphicsListRegistry);
      }
      else
      {
         reachabilityConstraintHandler = null;
         planarRegionConstraintProvider = null;
      }

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition predictedEndOfStateICP = new YoGraphicPosition(yoNamePrefix + "PredictedEndOfStateICP", this.predictedEndOfStateICP, 0.005,
                                                                       YoAppearance.MidnightBlue(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition clippedFootstepSolution = new YoGraphicPosition(yoNamePrefix + "ClippedFootstepSolution", this.footstepSolution.getPosition(), 0.005,
                                                                        YoAppearance.DarkRed(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition feedbackCoP = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(),
                                                            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      artifactList.add(predictedEndOfStateICP.createArtifact());
      artifactList.add(clippedFootstepSolution.createArtifact());
      artifactList.add(feedbackCoP.createArtifact());

      solutionHandler.setupVisualizers(artifactList);
      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /** {@inheritDoc} */
   @Override
   public void clearPlan()
   {
      numberOfRegisteredSteps.set(0);
      upcomingFootstep.setToZero();

      transferDuration.setToNaN();
      swingDuration.setToNaN();
      nextTransferDuration.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferDuration(double duration)
   {
      transferDuration.set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setSwingDuration(double duration)
   {
      swingDuration.set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setNextTransferDuration(double duration)
   {
      nextTransferDuration.set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration.set(finalTransferDuration);
   }

   /** {@inheritDoc} */
   @Override
   public void addFootstepToPlan(SimpleAdjustableFootstep footstep, double swingDuration, double transferDuration)
   {
      FramePose3DReadOnly footstepPose = footstep.getSoleFramePose();
      footstepPose.checkReferenceFrameMatch(worldFrame);
      if (!footstepPose.containsNaN())
      {
         if (numberOfRegisteredSteps.getValue() == 0)
         {
            upcomingFootstep.set(footstepPose);
            upcomingFootstepSide.set(footstep.getRobotSide());
            upcomingFootstepContactPoints.clear();
            ConvexPolygon2DReadOnly foothold = footstep.getFoothold();
            for (int i = 0; i < foothold.getNumberOfVertices(); i++)
            {
               upcomingFootstepContactPoints.add().set(foothold.getVertex(i));
            }

            footstepSolution.set(footstepPose);
            unclippedFootstepSolution.set(footstepPose.getPosition());

            this.swingDuration.set(swingDuration);
            this.transferDuration.set(transferDuration);

            footstepIsAdjustable.set(footstep.getIsAdjustable());
            useStepAdjustment.set(allowStepAdjustment.getValue() && footstepIsAdjustable.getBooleanValue());
         }
         else if (numberOfRegisteredSteps.getValue() == 1)
         {
            nextTransferDuration.set(transferDuration);
         }

         numberOfRegisteredSteps.increment();
      }
      else
      {
         LogTools.warn("Received bad footstep: " + footstep);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStationary.set(true);
      isInDoubleSupport.set(true);
      isICPStuck.set(false);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      solver.resetCoPLocationConstraint();
      solver.resetReachabilityConstraint();
      solver.resetPlanarRegionConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());
      if (reachabilityConstraintHandler != null)
      {
         solver.addReachabilityPolygon(reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport());
      }

      if (planarRegionConstraintProvider != null)
      {
         planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport();
      }

      solver.notifyResetActiveSet();

      transferDuration.set(finalTransferDuration.getDoubleValue());

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      speedUpTime.set(0.0);
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide)
   {
      this.transferToSide.set(transferToSide);
      isInDoubleSupport.set(true);
      isStationary.set(false);
      isICPStuck.set(false);

      if (numberOfRegisteredSteps.getValue() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      solver.resetCoPLocationConstraint();
      solver.resetReachabilityConstraint();
      solver.resetPlanarRegionConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());
      if (reachabilityConstraintHandler != null)
      {
         solver.addReachabilityPolygon(reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport());
      }

      if (planarRegionConstraintProvider != null)
         planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport();

      solver.notifyResetActiveSet();
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      if (!upcomingFootstepSide.getValue().equals(supportSide.getOppositeSide()))
      {
         throw new RuntimeException("Somehow initializing the wrong side!");
      }

      this.supportSide.set(supportSide);
      isStationary.set(false);
      isInDoubleSupport.set(false);
      isICPStuck.set(false);

      if (numberOfRegisteredSteps.getValue() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      solver.resetCoPLocationConstraint();
      solver.resetReachabilityConstraint();
      solver.resetPlanarRegionConstraint();

      solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide));
      if (reachabilityConstraintHandler != null)
      {
         solver.addReachabilityPolygon(reachabilityConstraintHandler.initializeReachabilityConstraintForSingleSupport(supportSide, upcomingFootstep));
      }

      if (planarRegionConstraintProvider != null)
      {
         planarRegionConstraintProvider.computeDistanceFromEdgeForNoOverhang(upcomingFootstepSide.getValue(), upcomingFootstepContactPoints);
         ConvexPolygon2D planarRegion = planarRegionConstraintProvider.updatePlanarRegionConstraintForSingleSupport(upcomingFootstepSide.getValue(),
                                                                                                                    upcomingFootstep,
                                                                                                                    upcomingFootstepContactPoints,
                                                                                                                    timeRemainingInState.getDoubleValue(),
                                                                                                                    currentICP, omega0);

         solver.setPlanarRegionConstraint(planarRegion);
      }

      solver.notifyResetActiveSet();
   }

   private void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      this.initialTime.set(initialTime);

      if (useFootstepRate.getValue())
      {
         if (useICPControlPolygons.getValue() && hasICPControlPolygons)
            icpControlPlane.projectPointOntoControlPlane(worldFrame, upcomingFootstep.getPosition(), projectedTempPoint3d);
         else
            projectedTempPoint3d.set(upcomingFootstep.getPosition());

         solver.resetFootstepRate(projectedTempPoint3d);
      }
   }

   private boolean computeWhetherToIncludeFootsteps()
   {
      if (!localUseStepAdjustment || isInDoubleSupport.getBooleanValue() || isStationary.getBooleanValue())
         return false;

      if (timeInCurrentState.getDoubleValue() / swingDuration.getDoubleValue() < fractionThroughSwingForAdjustment.getValue())
         return false;

      if (icpError.length() < Math.abs(minICPErrorForStepAdjustment.getValue()) && !includeFootsteps.getBooleanValue())
         return false;

      return numberOfRegisteredSteps.getValue() > 0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useStepAdjustment()
   {
      return useStepAdjustment.getBooleanValue();
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack)
   {
      desiredCMPToPack.set(feedbackCMP);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack)
   {
      desiredCoPToPack.set(feedbackCoP);
   }

   /** {@inheritDoc} */
   @Override
   public FramePose3DReadOnly getFootstepSolution()
   {
      return footstepSolution;
   }

   /** {@inheritDoc} */
   @Override
   public boolean wasFootstepAdjusted()
   {
      return solutionHandler.wasFootstepAdjusted();
   }

   private final FrameVector3D scaledAdjustment = new FrameVector3D();

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getICPShiftFromStepAdjustment()
   {
      scaledAdjustment.setIncludingFrame(solutionHandler.getTotalFootstepAdjustment(), 0.0);
      scaledAdjustment.scale(footstepMultiplier.getDoubleValue());

      return scaledAdjustment;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getValue();
   }

   private final FrameVector2D desiredCMPOffsetToThrowAway = new FrameVector2D();

   /** {@inheritDoc} */
   @Override
   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity, FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(currentTime, desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentICPVelocity, omega0);
   }

   /** {@inheritDoc} */
   @Override
   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity, FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset, FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0)
   {
      controllerTimer.startMeasurement();

      if (!initialized)
      {
         initialize();
         initialized = true;
      }

      solver.setConsiderAngularMomentumInAdjustment(considerAngularMomentumInAdjustment.getValue());
      solver.setConsiderFeedbackInAdjustment(considerFeedbackInAdjustment.getValue());

      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);
      this.perfectCoP.set(perfectCoP);
      this.perfectCMPOffset.set(perfectCMPOffset);
      this.currentICP.set(currentICP);
      this.currentICPVelocity.set(currentICPVelocity);

      this.desiredICP.changeFrame(worldFrame);
      this.desiredICPVelocity.changeFrame(worldFrame);
      this.perfectCoP.changeFrame(worldFrame);
      this.perfectCMPOffset.changeFrame(worldFrame);
      this.currentICP.changeFrame(worldFrame);
      this.currentICPVelocity.changeFrame(worldFrame);

      this.yoPerfectCoP.set(this.perfectCoP);
      this.yoPerfectCMP.add(this.perfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      boolean includeFootsteps = computeWhetherToIncludeFootsteps();
      // if we are switching between including footsteps and not, the decision variables are changing, so the active set needs resetting
      if (includeFootsteps != this.includeFootsteps.getBooleanValue())
      {
         solver.notifyResetActiveSet();
         this.includeFootsteps.set(includeFootsteps);
      }

      scaleStepRateWeightWithTime();
      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(omega0, includeFootsteps);

      solver.setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      qpSolverTimer.startMeasurement();
      boolean converged = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(converged, includeFootsteps);

      modifyCMPFeedbackWeightUsingIntegral();

      controllerTimer.stopMeasurement();
   }

   private void initialize()
   {
      scaledCMPFeedbackWeight.set(cmpFeedbackWeight.getValue());
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

   private void submitSolverTaskConditions(double omega0, boolean includeFootsteps)
   {
      if (isInDoubleSupport.getBooleanValue())
      {
         solver.resetCoPLocationConstraint();
         solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForDoubleSupport());

         if (copConstraintHandler.hasSupportPolygonChanged())
            solver.notifyResetActiveSet();
      }
      else
      {
         solver.resetCoPLocationConstraint();
         solver.addSupportPolygon(copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide.getEnumValue()));

         boolean resetActiveSet = copConstraintHandler.hasSupportPolygonChanged();

         if (planarRegionConstraintProvider != null)
         {
            ConvexPolygon2D planarRegionConstraint = planarRegionConstraintProvider.updatePlanarRegionConstraintForSingleSupport(upcomingFootstepSide.getValue(),
                                                                                                                                 upcomingFootstep,
                                                                                                                                 upcomingFootstepContactPoints,
                                                                                                                                 timeRemainingInState.getDoubleValue(),
                                                                                                                                 currentICP, omega0);

            solver.resetPlanarRegionConstraint();
            solver.setPlanarRegionConstraint(planarRegionConstraint);

            resetActiveSet = resetActiveSet || planarRegionConstraintProvider.hasConstraintChanged();
         }

         if (resetActiveSet)
            solver.notifyResetActiveSet();
      }

      solver.resetFootstepConditions();

      if (localUseStepAdjustment && !isInDoubleSupport.getBooleanValue())
      {
         submitFootstepTaskConditionsToSolver(omega0, includeFootsteps);

         solver.resetReachabilityConstraint();
         if (reachabilityConstraintHandler != null)
         {
            solver.addReachabilityPolygon(reachabilityConstraintHandler.updateReachabilityConstraint());
         }
      }
      else
      {
         predictedEndOfStateICP.setToNaN();
      }

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

   private void submitFootstepTaskConditionsToSolver(double omega0, boolean includeFootsteps)
   {
      if (includeFootsteps)
      {
         ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();
         helper.transformToWorldFrame(footstepWeights, forwardFootstepWeight.getValue(), lateralFootstepWeight.getValue(), soleFrame);
         yoFootstepWeights.set(footstepWeights);

         this.footstepMultiplier.set(computeFootstepAdjustmentMultiplier(omega0));

         predictedEndOfStateICP.sub(desiredICP, yoPerfectCMP);
         predictedEndOfStateICP.scaleAdd(Math.exp(omega0 * timeRemainingInState.getDoubleValue()), yoPerfectCMP);

         if (useICPControlPolygons.getValue() && hasICPControlPolygons)
            icpControlPlane.projectPointOntoControlPlane(worldFrame, upcomingFootstep.getPosition(), projectedTempPoint3d);
         else
            projectedTempPoint3d.set(upcomingFootstep.getPosition());

         footstepLocationSubmitted.set(projectedTempPoint3d);
         solver.setFootstepAdjustmentConditions(footstepMultiplier.getDoubleValue(), footstepWeights, footstepAdjustmentSafetyFactor.getValue(),
                                                projectedTempPoint3d);
      }

      if (useFootstepRate.getValue())
         solver.setFootstepRateWeight(scaledFootstepRateWeight.getDoubleValue() / controlDTSquare);
   }

   private double computeFootstepAdjustmentMultiplier(double omega0)
   {
      double timeInTransferForShifting = Math
            .min(maximumTimeFromTransfer.getValue(), transferDurationSplitFraction.getValue() * nextTransferDuration.getDoubleValue());
      recursionTime.set(Math.max(timeRemainingInState.getDoubleValue(), 0.0) + timeInTransferForShifting);
      recursionMultiplier.set(Math.exp(-omega0 * recursionTime.getDoubleValue()));

      double finalRecursionMultiplier = Math.exp(-omega0 * timeInTransferForShifting);

      double minimumFootstepMultiplier = Math.min(this.minimumFootstepMultiplier.getValue(), finalRecursionMultiplier);
      return minimumFootstepMultiplier + (1.0 - minimumFootstepMultiplier / finalRecursionMultiplier) * recursionMultiplier.getDoubleValue();
   }

   private boolean solveQP()
   {
      perfectCoP.set(yoPerfectCoP);
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

   private void extractSolutionsFromSolver(boolean converged, boolean includeFootsteps)
   {
      numberOfIterations.set(solver.getNumberOfIterations());

      // Don't pole the new solutions if the solver has not converged.
      if (converged)
      {
         if (localUseStepAdjustment && includeFootsteps)
         {
            if (planarRegionConstraintProvider != null)
            {
               PlanarRegion activePlanarRegion = planarRegionConstraintProvider.getActivePlanarRegion();
               solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootstep, activePlanarRegion, solver);
               boolean footstepWasAdjustedBySnapper = planarRegionConstraintProvider.snapFootPoseToActivePlanarRegion(footstepSolution);
               solutionHandler.setFootstepWasAdjustedBySnapper(footstepWasAdjustedBySnapper);
            }
            else
            {
               solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootstep, null, solver);
               solutionHandler.setFootstepWasAdjustedBySnapper(false);
            }
         }

         if (isInDoubleSupport.getBooleanValue())
            solutionHandler.resetAdjustment();

         solutionHandler.updateVisualizers(desiredICP, footstepMultiplier.getDoubleValue());

         solver.getCoPFeedbackDifference(feedbackCoPDelta);
         solver.getCMPFeedbackDifference(feedbackCMPDelta);
         solver.getDynamicsError(dynamicsError);

         if (COMPUTE_COST_TO_GO)
            solutionHandler.updateCostsToGo(solver);
      }
      else
      {
         if (localUseStepAdjustment && includeFootsteps)
            solutionHandler.zeroAdjustment();
      }

      isICPStuck.update(computeIsStuck());
      computeICPIntegralTerm();

      feedbackCoP.add(yoPerfectCoP, feedbackCoPDelta);
      feedbackCMP.add(feedbackCoP, perfectCMPOffset);
      feedbackCMP.add(feedbackCMPDelta);
      feedbackCMP.add(feedbackCMPIntegral);

      if (limitReachabilityFromAdjustment.getValue() && localUseStepAdjustment && includeFootsteps)
         updateReachabilityRegionFromAdjustment();

      if (wasFootstepAdjusted() && CONTINUOUSLY_UPDATE_DESIRED_POSITION)
         upcomingFootstep.set(footstepSolution);
   }

   private void updateReachabilityRegionFromAdjustment()
   {
      if (reachabilityConstraintHandler != null)
      {
         reachabilityConstraintHandler.updateReachabilityBasedOnAdjustment(upcomingFootstep, unclippedFootstepSolution, wasFootstepAdjusted());
      }
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

   private void scaleStepRateWeightWithTime()
   {
      if (scaleStepRateWeightWithTime.getValue())
      {
         double alpha = Math.max(timeRemainingInState.getDoubleValue(), minimumTimeRemaining.getValue()) / swingDuration.getDoubleValue();
         scaledFootstepRateWeight.set(footstepRateWeight.getValue() / alpha);
      }
      else
      {
         scaledFootstepRateWeight.set(footstepRateWeight.getValue());
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

      if ((currentICPVelocity.length() < thresholdForStuck.getValue()) && (timeRemainingInState.getDoubleValue() <= minimumTimeRemaining.getValue()))
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
   @Override
   public void submitCurrentPlanarRegions(List<PlanarRegion> planarRegions)
   {
      if (planarRegionConstraintProvider != null)
         planarRegionConstraintProvider.setPlanarRegions(planarRegions);
   }

   /** {@inheritDoc} */
   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.copConstraintHandler.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }
}
