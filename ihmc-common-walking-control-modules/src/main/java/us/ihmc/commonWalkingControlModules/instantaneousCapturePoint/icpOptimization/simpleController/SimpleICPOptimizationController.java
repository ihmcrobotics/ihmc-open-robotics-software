package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimpleICPOptimizationController implements ICPOptimizationController
{
   private static final boolean VISUALIZE = true;
   private static final boolean DEBUG = false;
   private static final boolean COMPUTE_COST_TO_GO = false;

   private static final boolean useICPControlPlane = true;

   private static final double footstepAdjustmentSafetyFactor = 1.0;
   private static final double transferSplitFraction = 0.3;

   private static final boolean useAngularMomentumIntegrator = true;
   private static final double angularMomentumIntegratorGain = 50.0;
   private static final double angularMomentumIntegratorLeakRatio = 0.92;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean useAngularMomentum = new YoBoolean(yoNamePrefix + "UseAngularMomentum", registry);

   private final YoBoolean scaleStepRegularizationWeightWithTime = new YoBoolean(yoNamePrefix + "ScaleStepRegularizationWeightWithTime", registry);
   private final YoBoolean scaleFeedbackWeightWithGain = new YoBoolean(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry);
   private final YoBoolean scaleUpcomingStepWeights = new YoBoolean(yoNamePrefix + "ScaleUpcomingStepWeights", registry);

   private final YoBoolean isStanding = new YoBoolean(yoNamePrefix + "IsStanding", registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   private final ArrayList<YoDouble> swingDurations = new ArrayList<>();
   private final ArrayList<YoDouble> transferDurations = new ArrayList<>();
   private final YoDouble transferDurationSplitFraction = new YoDouble(yoNamePrefix + "TransferDurationSplitFraction", registry);
   private final YoDouble finalTransferDuration = new YoDouble(yoNamePrefix + "FinalTransferDuration", registry);

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

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final YoFramePoint upcomingFootstepLocation;
   private final YoFramePoint2d footstepLocationSubmitted;
   private final YoFramePoint2d footstepSolution;
   private final YoFramePoint2d unclippedFootstepSolution;

   private final YoDouble forwardFootstepWeight = new YoDouble(yoNamePrefix + "ForwardFootstepWeight", registry);
   private final YoDouble lateralFootstepWeight = new YoDouble(yoNamePrefix + "LateralFootstepWeight", registry);
   private final YoFramePoint2d scaledFootstepWeights = new YoFramePoint2d(yoNamePrefix + "ScaledFootstepWeights", worldFrame, registry);

   private final YoDouble feedbackForwardWeight = new YoDouble(yoNamePrefix + "FeedbackForwardWeight", registry);
   private final YoDouble feedbackLateralWeight = new YoDouble(yoNamePrefix + "FeedbackLateralWeight", registry);
   private final YoFramePoint2d scaledFeedbackWeight = new YoFramePoint2d(yoNamePrefix + "ScaledFeedbackWeight", worldFrame, registry);

   private final YoDouble maxAllowedDistanceCMPSupport = new YoDouble(yoNamePrefix + "MaxAllowedDistanceCMPSupport", registry);
   private final YoDouble safeCoPDistanceToEdge = new YoDouble(yoNamePrefix + "SafeCoPDistanceToEdge", registry);

   private final YoDouble footstepRegularizationWeight = new YoDouble(yoNamePrefix + "FootstepRegularizationWeight", registry);
   private final YoDouble feedbackRegularizationWeight = new YoDouble(yoNamePrefix + "FeedbackRegularizationWeight", registry);
   private final YoDouble scaledFootstepRegularizationWeight = new YoDouble(yoNamePrefix + "ScaledFootstepRegularizationWeight", registry);
   private final YoDouble dynamicRelaxationWeight = new YoDouble(yoNamePrefix + "DynamicRelaxationWeight", registry);

   private final YoDouble angularMomentumMinimizationWeight = new YoDouble(yoNamePrefix + "AngularMomentumMinimizationWeight", registry);
   private final YoDouble scaledAngularMomentumMinimizationWeight = new YoDouble(yoNamePrefix + "ScaledAngularMomentumMinimizationWeight", registry);

   private final YoDouble cumulativeAngularMomentum = new YoDouble(yoNamePrefix + "CumulativeAngularMomentum", registry);

   private final YoBoolean limitReachabilityFromAdjustment = new YoBoolean(yoNamePrefix + "LimitReachabilityFromAdjustment", registry);

   private final YoDouble feedbackOrthogonalGain = new YoDouble(yoNamePrefix + "FeedbackOrthogonalGain", registry);
   private final YoDouble feedbackParallelGain = new YoDouble(yoNamePrefix + "FeedbackParallelGain", registry);

   private final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "FootstepMultiplier", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   private final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   private final PlanarRegionConstraintProvider planarRegionConstraintProvider;
   private final SimpleICPOptimizationSolutionHandler solutionHandler;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final ICPControlPlane icpControlPlane;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final boolean useFootstepRegularization;
   private final boolean useFeedbackRegularization;

   private final int maximumNumberOfFootstepsToConsider;

   private boolean localUseStepAdjustment;
   private boolean localScaleUpcomingStepWeights;

   private final FramePoint3D tempPoint3d = new FramePoint3D();
   private final FramePoint3D projectedTempPoint3d = new FramePoint3D();
   private final FramePoint2D tempPoint2d = new FramePoint2D();
   private final FrameVector2D tempVector2d = new FrameVector2D();

   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D perfectCMP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();

   private final SimpleICPOptimizationQPSolver solver;

   private final double controlDT;
   private final double dynamicRelaxationDoubleSupportWeightModifier;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   public SimpleICPOptimizationController(WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                          ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                          double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet,
           controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public SimpleICPOptimizationController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                          BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                          YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.contactableFeet = contactableFeet;
      this.icpControlPlane = icpControlPolygons.getIcpControlPlane();

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      dynamicRelaxationDoubleSupportWeightModifier = icpOptimizationParameters.getDynamicRelaxationDoubleSupportWeightModifier();

      useFootstepRegularization = icpOptimizationParameters.useFootstepRegularization();
      useFeedbackRegularization = icpOptimizationParameters.useFeedbackRegularization();

      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());
      useAngularMomentum.set(icpOptimizationParameters.useAngularMomentum());

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
      scaledAngularMomentumMinimizationWeight.set(icpOptimizationParameters.getAngularMomentumMinimizationWeight());
      limitReachabilityFromAdjustment.set(icpOptimizationParameters.getLimitReachabilityFromAdjustment());

      safeCoPDistanceToEdge.set(icpOptimizationParameters.getSafeCoPDistanceToEdge());
      if (walkingControllerParameters != null)
         maxAllowedDistanceCMPSupport.set(walkingControllerParameters.getMaxAllowedDistanceCMPSupport());
      else
         maxAllowedDistanceCMPSupport.setToNaN();

      minimumTimeRemaining.set(icpOptimizationParameters.getMinimumTimeRemaining());

      upcomingFootstepLocation = new YoFramePoint(yoNamePrefix + "UpcomingFootstepLocation", worldFrame, registry);
      footstepLocationSubmitted = new YoFramePoint2d(yoNamePrefix + "FootstepLocationSubmitted", worldFrame, registry);
      footstepSolution = new YoFramePoint2d(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
      unclippedFootstepSolution = new YoFramePoint2d(yoNamePrefix + "UnclippedFootstepSolutionLocation", worldFrame, registry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration" + i, registry);
         swingDuration.setToNaN();
         swingDurations.add(swingDuration);
         YoDouble transferDuration = new YoDouble(yoNamePrefix + "TransferDuration" + i, registry);
         transferDuration.setToNaN();
         transferDurations.add(transferDuration);
      }
      transferDurationSplitFraction.set(transferSplitFraction);

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      boolean updateRegularizationAutomatically = true;
      solver = new SimpleICPOptimizationQPSolver(icpOptimizationParameters, totalVertices, COMPUTE_COST_TO_GO, updateRegularizationAutomatically);

      solutionHandler = new SimpleICPOptimizationSolutionHandler(icpControlPlane, icpOptimizationParameters, useICPControlPlane, DEBUG, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons, useICPControlPlane);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, yoNamePrefix, VISUALIZE,
                                                                                       registry, yoGraphicsListRegistry);
      planarRegionConstraintProvider = new PlanarRegionConstraintProvider(walkingControllerParameters, bipedSupportPolygons, contactableFeet, yoNamePrefix,
                                                                          VISUALIZE, registry, yoGraphicsListRegistry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition predictedEndOfStateICP = new YoGraphicPosition(yoNamePrefix + "PredictedEndOfStateICP", this.predictedEndOfStateICP, 0.005, YoAppearance.MidnightBlue(),
                                                                       YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition clippedFootstepSolution = new YoGraphicPosition(yoNamePrefix + "ClippedFootstepSolution", this.footstepSolution, 0.005,
                                                                        YoAppearance.ForestGreen(), YoGraphicPosition.GraphicType.SOLID_BALL);
      solutionHandler.setupVisualizers(artifactList);

      artifactList.add(predictedEndOfStateICP.createArtifact());
      artifactList.add(clippedFootstepSolution.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void setFootstepWeights(double forwardWeight, double lateralWeight)
   {
      forwardFootstepWeight.set(forwardWeight);
      lateralFootstepWeight.set(lateralWeight);
   }

   @Override
   public void setFeedbackWeights(double forwardWeight, double lateralWeight)
   {
      feedbackForwardWeight.set(forwardWeight);
      feedbackLateralWeight.set(lateralWeight);
   }

   @Override
   public void clearPlan()
   {
      upcomingFootsteps.clear();
      upcomingFootstepLocation.setToZero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
      }
   }

   @Override
   public void setTransferDuration(int stepNumber, double duration)
   {
      int numberOfFootstepsRegistered = upcomingFootsteps.size();
      if (stepNumber < numberOfFootstepsRegistered + 1)
         transferDurations.get(stepNumber).set(duration);
   }

   @Override
   public void setTransferSplitFraction(int stepNumber, double splitFraction)
   {

   }

   @Override
   public void setSwingDuration(int stepNumber, double duration)
   {
      int numberOfFootstepsRegistered = upcomingFootsteps.size();
      if (stepNumber < numberOfFootstepsRegistered)
         swingDurations.get(stepNumber).set(duration);
   }

   @Override
   public void setSwingSplitFraction(int stepNumber, double splitFraction)
   {
   }

   @Override
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration.set(finalTransferDuration);
   }

   @Override
   public void setFinalTransferSplitFraction(double finalTransferSplitFraction)
   {
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
         {
            int footstepIndex = upcomingFootsteps.size();
            upcomingFootsteps.add(footstep);

            swingDurations.get(footstepIndex).set(timing.getSwingTime());
            transferDurations.get(footstepIndex).set(timing.getTransferTime());

            if (footstepIndex == 0)
            {
               footstep.getPosition(tempPoint3d);
               footstep.getPosition2d(tempPoint2d);
               upcomingFootstepLocation.set(tempPoint3d);
               footstepSolution.set(tempPoint2d);
               unclippedFootstepSolution.set(tempPoint2d);
            }
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
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);
      planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport(solver);

      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      speedUpTime.set(0.0);
   }

   @Override
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.transferToSide.set(transferToSide);
      isInDoubleSupport.set(true);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      footstepSolution.setToNaN();
      unclippedFootstepSolution.setToNaN();

      copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForDoubleSupport(solver);
      planarRegionConstraintProvider.updatePlanarRegionConstraintForDoubleSupport(solver);
   }

   @Override
   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInDoubleSupport.set(false);

      int numberOfFootstepRegistered = upcomingFootsteps.size();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

      initializeOnContactChange(initialTime);

      copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.initializeReachabilityConstraintForSingleSupport(supportSide, solver);

      Footstep upcomingFootstep = upcomingFootsteps.get(0);
      planarRegionConstraintProvider.setActivePlanarRegion(upcomingFootstep.getPlanarRegion());
      planarRegionConstraintProvider.computeDistanceFromEdgeForNoOverhang(upcomingFootstep);
      planarRegionConstraintProvider.updatePlanarRegionConstraintForSingleSupport(supportSide, timeRemainingInState.getDoubleValue(), currentICP, omega0, solver);
   }

   private void initializeOnContactChange(double initialTime)
   {
      speedUpTime.set(0.0);

      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();

      this.initialTime.set(initialTime);

      if (useFootstepRegularization)
      {
         int stepIndex = 0;
         upcomingFootsteps.get(stepIndex).getPosition(tempPoint3d);

         if (useICPControlPlane)
            icpControlPlane.projectPointOntoControlPlane(tempPoint3d, projectedTempPoint3d);
         else
            projectedTempPoint3d.set(tempPoint3d);

         tempPoint2d.set(projectedTempPoint3d);
         solver.resetFootstepRegularization(tempPoint2d);
      }

      solver.resetOnContactChange();
   }

   private int clipNumberOfFootstepsToConsiderToProblem()
   {
      int numberOfFootstepsToConsider = Math.min(1, upcomingFootsteps.size());

      if (!localUseStepAdjustment || isInDoubleSupport.getBooleanValue() || isStanding.getBooleanValue())
         numberOfFootstepsToConsider = 0;

      return numberOfFootstepsToConsider;
   }

   @Override
   public int getNumberOfFootstepsToConsider()
   {
      if (useStepAdjustment.getBooleanValue())
         return 1;
      else
         return 0;
   }

   @Override
   public void getDesiredCMP(FramePoint2D desiredCMP)
   {
      feedbackCMP.getFrameTuple2d(desiredCMP);
   }

   @Override
   public void getFootstepSolution(int footstepIndex, FramePoint2D footstepSolutionToPack)
   {
      if (footstepIndex > 0)
         throw new RuntimeException("This controller is not set up to handle more than one footstep.");

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

      //updateYoFootsteps();

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem();

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      submitSolverTaskConditions(numberOfFootstepsToConsider, omega0);

      qpSolverTimer.startMeasurement();
      NoConvergenceException noConvergenceException = solveQP();
      qpSolverTimer.stopMeasurement();

      extractSolutionsFromSolver(numberOfFootstepsToConsider, noConvergenceException);

      modifyAngularMomentumWeightUsingIntegral();

      controllerTimer.stopMeasurement();
   }

   @Override
   public void setFinalTransferSplitFractionToDefault()
   {
   }

   @Override
   public void setReferenceICPVelocity(FrameVector2D referenceICPVelocity)
   {
   }

   @Override
   public double getOptimizedTimeRemaining()
   {
      throw new RuntimeException("This is not implemented in this solver.");
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

   private int submitSolverTaskConditions(int numberOfFootstepsToConsider, double omega0)
   {
      if (isInDoubleSupport.getBooleanValue())
      {
         copConstraintHandler.updateCoPConstraintForDoubleSupport(solver);
      }
      else
      {
         copConstraintHandler.updateCoPConstraintForSingleSupport(supportSide.getEnumValue(), solver);
         planarRegionConstraintProvider
               .updatePlanarRegionConstraintForSingleSupport(supportSide.getEnumValue(), timeRemainingInState.getDoubleValue(), currentICP, omega0, solver);
      }

      solver.resetFootstepConditions();

      if (localUseStepAdjustment && !isInDoubleSupport.getBooleanValue())
      {
         submitFootstepTaskConditionsToSolver(numberOfFootstepsToConsider, omega0);
         reachabilityConstraintHandler.updateReachabilityConstraint(solver);
      }
      else
      {
         predictedEndOfStateICP.setToNaN();
      }

      submitFeedbackTaskConditionsToSolver();
      submitAngularMomentumTaskConditionsToSolver();

      return numberOfFootstepsToConsider;
   }

   private void submitFeedbackTaskConditionsToSolver()
   {
      helper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);

      double dynamicRelaxationWeight = this.dynamicRelaxationWeight.getDoubleValue();
      if (isInDoubleSupport.getBooleanValue())
         dynamicRelaxationWeight = dynamicRelaxationWeight / dynamicRelaxationDoubleSupportWeightModifier;

      solver.resetFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), tempVector2d.getX(), tempVector2d.getY(), dynamicRelaxationWeight);
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

   private void submitFootstepTaskConditionsToSolver(int numberOfFootstepsToConsider, double omega0)
   {
      for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
      {
         ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();
         helper.transformToWorldFrame(tempVector2d, forwardFootstepWeight, lateralFootstepWeight, soleFrame);
         scaledFootstepWeights.set(tempVector2d);

         if (localScaleUpcomingStepWeights)
            scaledFootstepWeights.scale(1.0 / (footstepIndex + 1));

         double recursionTime = timeRemainingInState.getDoubleValue() + transferDurationSplitFraction.getDoubleValue() * transferDurations.get(1).getDoubleValue();
         double recursionMultiplier = Math.exp(-omega0 * recursionTime);
         this.footstepMultiplier.set(recursionMultiplier);

         yoPerfectCMP.getFrameTuple2d(perfectCMP);
         predictedEndOfStateICP.set(desiredICP);
         predictedEndOfStateICP.sub(perfectCMP);
         predictedEndOfStateICP.scale(Math.exp(omega0 * timeRemainingInState.getDoubleValue()));
         predictedEndOfStateICP.add(perfectCMP);

         /*
         Footstep upcomingFootstep = upcomingFootsteps.get(footstepIndex);
         upcomingFootstep.getPosition(tempPoint3d);
         */
         upcomingFootstepLocation.getFrameTuple(tempPoint3d);
         if (useICPControlPlane)
            icpControlPlane.projectPointOntoControlPlane(tempPoint3d, projectedTempPoint3d);
         else
            projectedTempPoint3d.set(tempPoint3d);
         tempPoint2d.set(projectedTempPoint3d);

         footstepLocationSubmitted.set(tempPoint2d);
         solver.setFootstepAdjustmentConditions(recursionMultiplier, scaledFootstepWeights.getX(), scaledFootstepWeights.getY(), footstepAdjustmentSafetyFactor,
                                                tempPoint2d);
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

   private void extractSolutionsFromSolver(int numberOfFootstepsToConsider, NoConvergenceException noConvergenceException)
   {
      // don't pole the new solutions if there's a no convergence exception
      if (noConvergenceException == null)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment && numberOfFootstepsToConsider > 0)
            solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootsteps.get(0), numberOfFootstepsToConsider, solver);

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

      if (limitReachabilityFromAdjustment.getBooleanValue())
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

      if (!useAngularMomentumIntegrator)
      {
         scaledAngularMomentumMinimizationWeight.set(angularMomentumMinimizationWeight);
         return;
      }

      double angularMomentumMagnitude = cmpCoPDifferenceSolution.length();

      double cumulativeAngularMomentumAfterLeak = angularMomentumMagnitude * controlDT + angularMomentumIntegratorLeakRatio * cumulativeAngularMomentum.getDoubleValue();
      cumulativeAngularMomentum.set(cumulativeAngularMomentumAfterLeak);

      double multiplier = 1.0 + angularMomentumIntegratorGain * cumulativeAngularMomentumAfterLeak;

      scaledAngularMomentumMinimizationWeight.set(multiplier * angularMomentumMinimizationWeight);
   }
}
