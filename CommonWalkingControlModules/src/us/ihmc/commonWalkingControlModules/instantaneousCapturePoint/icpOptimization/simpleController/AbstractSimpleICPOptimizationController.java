package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public abstract class AbstractSimpleICPOptimizationController implements ICPOptimizationController
{
   protected static final boolean VISUALIZE = true;
   protected static final boolean DEBUG = false;
   protected static final boolean COMPUTE_COST_TO_GO = false;

   private static final double footstepAdjustmentSafetyFactor = 1.0;
   private static final double transferSplitFraction = 0.5;

   private static final boolean useAngularMomentumIntegrator = true;
   private static final double angularMomentumIntegratorGain = 50.0;
   private static final double angularMomentumIntegratorLeakRatio = 0.92;

   protected static final String yoNamePrefix = "controller";
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean useAngularMomentum = new YoBoolean(yoNamePrefix + "UseAngularMomentum", registry);

   private final YoBoolean scaleStepRegularizationWeightWithTime = new YoBoolean(yoNamePrefix + "ScaleStepRegularizationWeightWithTime", registry);
   private final YoBoolean scaleFeedbackWeightWithGain = new YoBoolean(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry);
   protected final YoBoolean scaleUpcomingStepWeights = new YoBoolean(yoNamePrefix + "ScaleUpcomingStepWeights", registry);

   protected final YoBoolean isStanding = new YoBoolean(yoNamePrefix + "IsStanding", registry);
   protected final YoBoolean isInDoubleSupport = new YoBoolean(yoNamePrefix + "IsInDoubleSupport", registry);

   protected final ArrayList<YoDouble> swingDurations = new ArrayList<>();
   protected final ArrayList<YoDouble> transferDurations = new ArrayList<>();
   protected final YoDouble transferDurationSplitFraction = new YoDouble(yoNamePrefix + "TransferDurationSplitFraction", registry);
   protected final YoDouble finalTransferDuration = new YoDouble(yoNamePrefix + "FinalTransferDuration", registry);

   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>(yoNamePrefix + "TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   protected final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   protected final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   protected final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);
   private final YoDouble minimumTimeRemaining = new YoDouble(yoNamePrefix + "MinimumTimeRemaining", registry);

   protected final YoFrameVector2d icpError = new YoFrameVector2d(yoNamePrefix + "ICPError", "", worldFrame, registry);
   protected final YoFramePoint2d feedbackCMP = new YoFramePoint2d(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   protected final YoFramePoint2d yoPerfectCMP = new YoFramePoint2d(yoNamePrefix + "PerfectCMP", worldFrame, registry);
   protected final YoFramePoint2d predictedEndOfStateICP = new YoFramePoint2d(yoNamePrefix + "PredictedEndOfStateICP", worldFrame, registry);

   protected final YoFrameVector2d feedbackCoPDelta = new YoFrameVector2d(yoNamePrefix + "FeedbackCoPDeltaSolution", worldFrame, registry);
   protected final YoFrameVector2d cmpCoPDifferenceSolution = new YoFrameVector2d(yoNamePrefix + "CMPCoPDifferenceSolution", "", worldFrame, registry);
   protected final YoFramePoint2d feedbackCMPDelta = new YoFramePoint2d(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);

   protected final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   protected final ArrayList<YoFramePoint2d> upcomingFootstepLocations = new ArrayList<>();
   protected final ArrayList<YoFramePoint2d> footstepSolutions = new ArrayList<>();
   protected final ArrayList<FramePoint2d> unclippedFootstepSolutions = new ArrayList<>();

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

   protected final YoBoolean limitReachabilityFromAdjustment = new YoBoolean(yoNamePrefix + "LimitReachabilityFromAdjustment", registry);

   private final YoDouble feedbackOrthogonalGain = new YoDouble(yoNamePrefix + "FeedbackOrthogonalGain", registry);
   private final YoDouble feedbackParallelGain = new YoDouble(yoNamePrefix + "FeedbackParallelGain", registry);

   protected final YoInteger numberOfIterations = new YoInteger(yoNamePrefix + "NumberOfIterations", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "FootstepMultiplier", registry);

   protected final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   protected final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   protected final SimpleICPOptimizationSolutionHandler solutionHandler;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   protected final ExecutionTimer qpSolverTimer = new ExecutionTimer("icpQPSolverTimer", 0.5, registry);
   protected final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final boolean useFootstepRegularization;
   private final boolean useFeedbackRegularization;

   private final int maximumNumberOfFootstepsToConsider;

   protected boolean localUseStepAdjustment;
   protected boolean localScaleUpcomingStepWeights;

   private final FramePoint2d tempPoint2d = new FramePoint2d();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   protected final FramePoint2d currentICP = new FramePoint2d();
   protected final FramePoint2d desiredICP = new FramePoint2d();
   protected final FramePoint2d perfectCMP = new FramePoint2d();
   protected final FrameVector2d desiredICPVelocity = new FrameVector2d();

   protected final SimpleICPOptimizationQPSolver solver;

   protected final double controlDT;
   protected final double dynamicRelaxationDoubleSupportWeightModifier;

   public AbstractSimpleICPOptimizationController(WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
                                                  ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                                  double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet,
           controlDT, yoGraphicsListRegistry);
   }

   public AbstractSimpleICPOptimizationController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                                  BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                                  SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.contactableFeet = contactableFeet;

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

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         upcomingFootstepLocations.add(new YoFramePoint2d(yoNamePrefix + "UpcomingFootstepLocation" + i, worldFrame, registry));
         footstepSolutions.add(new YoFramePoint2d(yoNamePrefix + "FootstepSolutionLocation" + i, worldFrame, registry));
         unclippedFootstepSolutions.add(new FramePoint2d(worldFrame));

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

      solutionHandler = new SimpleICPOptimizationSolutionHandler(icpOptimizationParameters, DEBUG, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, icpControlPolygons);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, yoNamePrefix, VISUALIZE,
                                                                                       registry, yoGraphicsListRegistry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);
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

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         upcomingFootstepLocations.get(i).setToZero();

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
            footstep.getPosition2d(tempPoint2d);
            upcomingFootstepLocations.get(footstepIndex).set(tempPoint2d);

            footstepSolutions.get(footstepIndex).set(tempPoint2d);
            unclippedFootstepSolutions.get(footstepIndex).set(tempPoint2d);

            swingDurations.get(footstepIndex).set(timing.getSwingTime());
            transferDurations.get(footstepIndex).set(timing.getTransferTime());
         }
         else
         {
            PrintTools.warn(this, "Received bad footstep: " + footstep);
         }
      }
   }

   protected void updateYoFootsteps()
   {
      for (int footstepIndex = 0; footstepIndex < upcomingFootsteps.size(); footstepIndex++)
      {
         upcomingFootsteps.get(footstepIndex).getPosition2d(tempPoint2d);
         upcomingFootstepLocations.get(footstepIndex).set(tempPoint2d);
      }

   }

   @Override
   public abstract void initializeForStanding(double initialTime);

   @Override
   public abstract void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0);

   @Override
   public abstract void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0);

   protected void initializeOnContactChange(double initialTime)
   {
      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();

      this.initialTime.set(initialTime);

      if (useFootstepRegularization)
      {
         int stepIndex = 0;
         upcomingFootsteps.get(stepIndex).getPosition2d(tempPoint2d);
         solver.resetFootstepRegularization(tempPoint2d);
      }

      solver.resetOnContactChange();
   }

   protected int clipNumberOfFootstepsToConsiderToProblem()
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
   public void getDesiredCMP(FramePoint2d desiredCMP)
   {
      feedbackCMP.getFrameTuple2d(desiredCMP);
   }

   @Override
   public void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack)
   {
      footstepSolutions.get(footstepIndex).getFrameTuple2d(footstepSolutionToPack);
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
   public abstract void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d perfectCMP, FramePoint2d currentICP, double omega0);

   @Override
   public void setFinalTransferSplitFractionToDefault()
   {
   }

   @Override
   public void setReferenceICPVelocity(FrameVector2d referenceICPVelocity)
   {
   }

   @Override
   public abstract double getOptimizedTimeRemaining();


   @Override
   public abstract void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   protected int submitSolverTaskConditions(int numberOfFootstepsToConsider, double omega0)
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
      ICPOptimizationControllerHelper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);

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
         ICPOptimizationControllerHelper.transformToWorldFrame(tempVector2d, forwardFootstepWeight, lateralFootstepWeight, soleFrame);
         scaledFootstepWeights.set(tempVector2d);

         if (localScaleUpcomingStepWeights)
            scaledFootstepWeights.scale(1.0 / (footstepIndex + 1));

         double recursionTime = timeRemainingInState.getDoubleValue() + transferDurationSplitFraction.getDoubleValue() * transferDurations.get(1).getDoubleValue();
         double recursionMultiplier = Math.exp(-omega0 * recursionTime);
         this.footstepMultiplier.set(recursionMultiplier);

         predictedEndOfStateICP.set(desiredICP);
         predictedEndOfStateICP.sub(perfectCMP);
         predictedEndOfStateICP.scale(Math.exp(omega0 * timeRemainingInState.getDoubleValue()));
         predictedEndOfStateICP.add(perfectCMP);

         upcomingFootsteps.get(footstepIndex).getPosition2d(tempPoint2d);
         solver.setFootstepAdjustmentConditions(recursionMultiplier, scaledFootstepWeights.getX(), scaledFootstepWeights.getY(), footstepAdjustmentSafetyFactor,
                                                tempPoint2d);
      }

      if (useFootstepRegularization)
         solver.setFootstepRegularizationWeight(scaledFootstepRegularizationWeight.getDoubleValue() / controlDT);
   }

   protected NoConvergenceException solveQP()
   {
      NoConvergenceException noConvergenceException = null;
      try
      {
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

   protected void extractSolutionsFromSolver(int numberOfFootstepsToConsider, NoConvergenceException noConvergenceException)
   {
      // don't pole the new solutions if there's a no convergence exception
      if (noConvergenceException == null)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment && numberOfFootstepsToConsider > 0)
            solutionHandler.extractFootstepSolutions(footstepSolutions, unclippedFootstepSolutions, upcomingFootsteps, numberOfFootstepsToConsider, solver);

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

      yoPerfectCMP.set(perfectCMP);
      feedbackCMP.set(perfectCMP);
      feedbackCMP.add(feedbackCMPDelta);

      if (limitReachabilityFromAdjustment.getBooleanValue())
         updateReachabilityRegionFromAdjustment();
   }

   private void updateReachabilityRegionFromAdjustment()
   {
      reachabilityConstraintHandler.updateReachabilityBasedOnAdjustment(upcomingFootsteps, unclippedFootstepSolutions, wasFootstepAdjusted());
   }

   protected void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
   }

   protected void computeTimeRemainingInState()
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

   protected void scaleStepRegularizationWeightWithTime()
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

   protected void scaleFeedbackWeightWithGain()
   {
      ReferenceFrame soleFrame = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame();

      //ICPOptimizationControllerHelper.transformToWorldFrame(tempVector2d, feedbackForwardWeight, feedbackLateralWeight, soleFrame);
      ICPOptimizationControllerHelper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackForwardWeight, feedbackLateralWeight);
      scaledFeedbackWeight.set(tempVector2d);

      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         ICPOptimizationControllerHelper.transformFromDynamicsFrame(tempVector2d, desiredICPVelocity, feedbackParallelGain, feedbackOrthogonalGain);
         scaledFeedbackWeight.scale(1.0 / tempVector2d.length());
      }
   }

   protected void modifyAngularMomentumWeightUsingIntegral()
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
