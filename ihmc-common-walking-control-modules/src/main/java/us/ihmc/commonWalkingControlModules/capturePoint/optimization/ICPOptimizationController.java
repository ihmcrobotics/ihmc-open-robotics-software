package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
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
   private static final boolean VISUALIZE = false;
   private static final boolean DEBUG = false;
   private static final boolean COMPUTE_COST_TO_GO = false;

   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanProvider allowStepAdjustment;
   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
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

   private final List<Footstep> upcomingFootsteps = new ArrayList<>();

   private final YoFramePose3D upcomingFootstepLocation = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepLocation", worldFrame, registry);
   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D footstepLocationSubmitted = new YoFramePoint2D(yoNamePrefix + "FootstepLocationSubmitted", worldFrame, registry);
   private final YoFramePoint2D unclippedFootstepSolution = new YoFramePoint2D(yoNamePrefix + "UnclippedFootstepSolutionLocation", worldFrame, registry);

   private final DoubleProvider minICPErrorForStepAdjustment;
   private final DoubleProvider fractionThroughSwingForAdjustment;

   private final DoubleProvider footstepAdjustmentSafetyFactor;
   private final DoubleProvider forwardFootstepWeight;
   private final DoubleProvider lateralFootstepWeight;
   private final YoFrameVector2D footstepWeights = new YoFrameVector2D(yoNamePrefix + "FootstepWeights", worldFrame, registry);

   private final DoubleProvider copFeedbackForwardWeight;
   private final DoubleProvider copFeedbackLateralWeight;
   private final DoubleProvider cmpFeedbackWeight;
   private final YoFrameVector2D scaledCoPFeedbackWeight = new YoFrameVector2D(yoNamePrefix + "ScaledCoPFeedbackWeight", worldFrame, registry);
   private final YoDouble scaledCMPFeedbackWeight = new YoDouble(yoNamePrefix + "ScaledCMPFeedbackWeight", registry);

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
   private final boolean hasICPControlPoygons;

   private final ICPControlGainsReadOnly feedbackGains;

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "FootstepMultiplier", registry);

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

   private final FramePose3D tmpPose = new FramePose3D();
   private final FramePoint3D tempPoint3d = new FramePoint3D();
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

   private final DoubleProvider minimumFeedbackWeight;
   private final DoubleProvider minimumFootstepWeight;
   private final BooleanProvider considerAngularMomentumInAdjustment;
   private final BooleanProvider considerFeedbackInAdjustment;

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet,
           controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public ICPOptimizationController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                    BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;
      this.contactableFeet = contactableFeet;

      if (icpControlPolygons != null)
         this.icpControlPlane = icpControlPolygons.getIcpControlPlane();
      else
         this.icpControlPlane = null;
      hasICPControlPoygons = this.icpControlPlane != null;

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

      minICPErrorForStepAdjustment = new DoubleParameter(yoNamePrefix + "MinICPErrorForStepAdjustment", registry, icpOptimizationParameters.getMinICPErrorForStepAdjustment());
      fractionThroughSwingForAdjustment = new DoubleParameter(yoNamePrefix + "FractionThroughSwingForAdjustment", registry, icpOptimizationParameters.getFractionThroughSwingForAdjustment());

      footstepAdjustmentSafetyFactor = new DoubleParameter(yoNamePrefix + "FootstepAdjustmentSafetyFactor", registry,
                                                           icpOptimizationParameters.getFootstepAdjustmentSafetyFactor());
      forwardFootstepWeight = new DoubleParameter(yoNamePrefix + "ForwardFootstepWeight", registry, icpOptimizationParameters.getForwardFootstepWeight());
      lateralFootstepWeight = new DoubleParameter(yoNamePrefix + "LateralFootstepWeight", registry, icpOptimizationParameters.getLateralFootstepWeight());
      footstepRateWeight = new DoubleParameter(yoNamePrefix + "FootstepRateWeight", registry, icpOptimizationParameters.getFootstepRateWeight());

      copFeedbackForwardWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackForwardWeight", registry, icpOptimizationParameters.getFeedbackForwardWeight());
      copFeedbackLateralWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackLateralWeight", registry, icpOptimizationParameters.getFeedbackLateralWeight());

      copCMPFeedbackRateWeight = new DoubleParameter(yoNamePrefix + "CoPCMPFeedbackRateWeight", registry, icpOptimizationParameters.getCoPCMPFeedbackRateWeight());
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

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRateAutomatically = true;
      solver = new ICPOptimizationQPSolver(totalVertices, COMPUTE_COST_TO_GO, updateRateAutomatically);

      minimumFeedbackWeight = new DoubleParameter(yoNamePrefix + "MinimumFeedbackWeight", registry, icpOptimizationParameters.getMinimumFeedbackWeight());
      minimumFootstepWeight = new DoubleParameter(yoNamePrefix + "MinimumFootstepWeight", registry, icpOptimizationParameters.getMinimumFootstepWeight());
      considerAngularMomentumInAdjustment = new BooleanParameter(yoNamePrefix + "ConsiderAngularMomentumInAdjustment", registry,
                                                                 icpOptimizationParameters.considerAngularMomentumInAdjustment());
      considerFeedbackInAdjustment = new BooleanParameter(yoNamePrefix + "ConsiderFeedbackInAdjustment", registry,
                                                          icpOptimizationParameters.considerFeedbackInAdjustment());

      solutionHandler = new ICPOptimizationSolutionHandler(icpControlPlane, icpOptimizationParameters, useICPControlPolygons, DEBUG, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons, useICPControlPolygons, hasICPControlPoygons,
                                                                     registry);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons,
                                                                                       walkingControllerParameters.getSteppingParameters(), yoNamePrefix,
                                                                                       VISUALIZE, registry, yoGraphicsListRegistry);
      if (walkingControllerParameters != null)
         planarRegionConstraintProvider = new PlanarRegionConstraintProvider(icpControlPlane, walkingControllerParameters, icpOptimizationParameters,
                                                                             bipedSupportPolygons, contactableFeet, yoNamePrefix, VISUALIZE, registry,
                                                                             yoGraphicsListRegistry);
      else
         planarRegionConstraintProvider = null;

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
                                                                        YoAppearance.ForestGreen(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition feedbackCoP = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(),
                                                            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      artifactList.add(predictedEndOfStateICP.createArtifact());
      artifactList.add(clippedFootstepSolution.createArtifact());
      artifactList.add(feedbackCoP.createArtifact());

      solutionHandler.setupVisualizers(artifactList);
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
               footstep.getPose(tmpPose);
               tmpPose.changeFrame(worldFrame);
               upcomingFootstepLocation.set(tmpPose);
               footstepSolution.set(tmpPose);
               unclippedFootstepSolution.set(tmpPose.getPosition());

               swingDuration.set(timing.getSwingTime());
               transferDuration.set(timing.getTransferTime());

               useStepAdjustment.set(allowStepAdjustment.getValue() && footstep.getIsAdjustable());
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

   @Override
   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStationary.set(true);
      isInDoubleSupport.set(true);
      isICPStuck.set(false);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);
      if (planarRegionConstraintProvider != null)
         planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport(solver);

      transferDuration.set(finalTransferDuration.getDoubleValue());

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      speedUpTime.set(0.0);
   }

   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.transferToSide.set(transferToSide);
      isInDoubleSupport.set(true);
      isStationary.set(false);
      isICPStuck.set(false);

      if (upcomingFootsteps.size() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);
      if (planarRegionConstraintProvider != null)
         planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport(solver);
   }

   @Override
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      this.supportSide.set(supportSide);
      isStationary.set(false);
      isInDoubleSupport.set(false);
      isICPStuck.set(false);

      if (upcomingFootsteps.size() < 2)
         nextTransferDuration.set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForSingleSupport(supportSide, solver);

      Footstep upcomingFootstep = upcomingFootsteps.get(0);

      if (planarRegionConstraintProvider != null)
      {
         planarRegionConstraintProvider.computeDistanceFromEdgeForNoOverhang(upcomingFootstep);
         planarRegionConstraintProvider
               .updatePlanarRegionConstraintForSingleSupport(upcomingFootstep, timeRemainingInState.getDoubleValue(), currentICP, omega0, solver);
      }
   }

   private void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();

      this.initialTime.set(initialTime);

      if (useFootstepRate.getValue())
      {
         upcomingFootsteps.get(0).getPosition(tempPoint3d);

         if (useICPControlPolygons.getValue() && hasICPControlPoygons)
            icpControlPlane.projectPointOntoControlPlane(worldFrame, tempPoint3d, projectedTempPoint3d);
         else
            projectedTempPoint3d.set(tempPoint3d);

         solver.resetFootstepRate(projectedTempPoint3d);
      }
   }

   private boolean computeWhetherToIncludeFootsteps()
   {
      if (!localUseStepAdjustment || isInDoubleSupport.getBooleanValue() || isStationary.getBooleanValue())
         return false;

      if (icpError.length() < Math.abs(minICPErrorForStepAdjustment.getValue()))
         return false;

      if (timeInCurrentState.getDoubleValue() / swingDuration.getDoubleValue() < fractionThroughSwingForAdjustment.getValue())
         return false;

      return upcomingFootsteps.size() > 0;
   }

   @Override
   public boolean useStepAdjustment()
   {
      return useStepAdjustment.getBooleanValue();
   }

   @Override
   public void getDesiredCMP(FramePoint2D desiredCMPToPack)
   {
      desiredCMPToPack.set(feedbackCMP);
   }

   @Override
   public void getFootstepSolution(Footstep footstepSolutionToPack)
   {
      footstepSolutionToPack.setPose(footstepSolution);
   }

   @Override
   public boolean wasFootstepAdjusted()
   {
      return solutionHandler.wasFootstepAdjusted();
   }

   @Override
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getValue();
   }

   private final FrameVector2D desiredCMPOffsetToThrowAway = new FrameVector2D();

   @Override
   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity, FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(currentTime, desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentICPVelocity, omega0);
   }

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

      solver.setMinimumFeedbackWeight(minimumFeedbackWeight.getValue());
      solver.setMinimumFootstepWeight(minimumFootstepWeight.getValue());
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

      scaleStepRateWeightWithTime();
      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(omega0, includeFootsteps);

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
         if (planarRegionConstraintProvider != null)
            planarRegionConstraintProvider
                  .updatePlanarRegionConstraintForSingleSupport(upcomingFootsteps.get(0), timeRemainingInState.getDoubleValue(), currentICP, omega0, solver);
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

      submitCoPFeedbackTaskConditionsToSolver();
      if (useCMPFeedback.getValue())
         submitCMPFeedbackTaskConditionsToSolver();
   }

   private void submitCoPFeedbackTaskConditionsToSolver()
   {
      helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackGains.getKpParallelToMotion(), feedbackGains.getKpOrthogonalToMotion());

      double dynamicsObjectiveWeight = this.dynamicsObjectiveWeight.getValue();
      if (isInDoubleSupport.getBooleanValue())
         dynamicsObjectiveWeight = dynamicsObjectiveWeight / dynamicsObjectiveDoubleSupportWeightModifier.getValue();

      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledCoPFeedbackWeight.getX(), scaledCoPFeedbackWeight.getY(), tempVector2d.getX(), tempVector2d.getY(),
                                   dynamicsObjectiveWeight);
      solver.setMaxCMPDistanceFromEdge(maxAllowedDistanceCMPSupport.getValue());
      solver.setCopSafeDistanceToEdge(safeCoPDistanceToEdge.getValue());

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

         double recursionTime =
               Math.max(timeRemainingInState.getDoubleValue(), 0.0) + transferDurationSplitFraction.getValue() * nextTransferDuration.getDoubleValue();
         double recursionMultiplier = Math.exp(-omega0 * recursionTime);
         this.footstepMultiplier.set(recursionMultiplier);

         predictedEndOfStateICP.sub(desiredICP, yoPerfectCMP);
         predictedEndOfStateICP.scaleAdd(Math.exp(omega0 * timeRemainingInState.getDoubleValue()), yoPerfectCMP);

         if (useICPControlPolygons.getValue() && hasICPControlPoygons)
            icpControlPlane.projectPointOntoControlPlane(worldFrame, upcomingFootstepLocation.getPosition(), projectedTempPoint3d);
         else
            projectedTempPoint3d.set(upcomingFootstepLocation.getPosition());

         footstepLocationSubmitted.set(projectedTempPoint3d);
         solver.setFootstepAdjustmentConditions(recursionMultiplier, footstepWeights.getX(), footstepWeights.getY(), footstepAdjustmentSafetyFactor.getValue(),
                                                projectedTempPoint3d);
      }

      if (useFootstepRate.getValue())
         solver.setFootstepRateWeight(scaledFootstepRateWeight.getDoubleValue() / controlDTSquare);
   }

   private boolean solveQP()
   {
      perfectCoP.set(yoPerfectCoP);
      boolean converged = solver.compute(icpError, perfectCoP, perfectCMPOffset);
      if (!converged)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            PrintTools.warn(this, "The QP has not converged. Only showing this once if it happens repeatedly.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();
      }

      return converged;
   }

   private void extractSolutionsFromSolver(boolean converged, boolean includeFootsteps)
   {
      // Don't pole the new solutions if the solver has not converged.
      if (converged)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment && includeFootsteps)
         {
            if (planarRegionConstraintProvider != null)
            {
               PlanarRegion activePlanarRegion = planarRegionConstraintProvider.getActivePlanarRegion();
               solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootsteps.get(0), activePlanarRegion, solver);
               boolean footstepWasAdjustedBySnapper = planarRegionConstraintProvider.snapFootPoseToActivePlanarRegion(footstepSolution);
               solutionHandler.setFootstepWasAdjustedBySnapper(footstepWasAdjustedBySnapper);
            }
            else
            {
               solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootsteps.get(0), null, solver);
               solutionHandler.setFootstepWasAdjustedBySnapper(false);
            }
         }

         if (isInDoubleSupport.getBooleanValue())
            solutionHandler.zeroAdjustment();

         solutionHandler.updateVisualizers(desiredICP, footstepMultiplier.getDoubleValue());

         solver.getCoPFeedbackDifference(feedbackCoPDelta);
         solver.getCMPFeedbackDifference(feedbackCMPDelta);
         solver.getDynamicsError(dynamicsError);

         if (COMPUTE_COST_TO_GO)
            solutionHandler.updateCostsToGo(solver);
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
         upcomingFootstepLocation.set(footstepSolution);
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

      if (scaleFeedbackWeightWithGain.getValue())
      {
         helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackGains.getKpParallelToMotion(), feedbackGains.getKpOrthogonalToMotion());
         scaledCoPFeedbackWeight.scale(1.0 / tempVector2d.length());
      }
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

   @Override
   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {
      if (planarRegionConstraintProvider != null)
         planarRegionConstraintProvider.setPlanarRegions(planarRegions);
   }

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.copConstraintHandler.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }
}
