package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.io.printing.PrintTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class ICPOptimizationController
{
   private static final boolean VISUALIZE = false;
   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);

   private final BooleanYoVariable useTwoCMPsInControl = new BooleanYoVariable("useTwoCMPsInControl", registry);
   private final BooleanYoVariable useInitialICP = new BooleanYoVariable("useInitialICP", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);
   private final BooleanYoVariable useFootstepRegularization = new BooleanYoVariable("useFootstepRegularization", registry);
   private final BooleanYoVariable useFeedbackRegularization = new BooleanYoVariable("useFeedbackRegularization", registry);
   private final BooleanYoVariable useFeedbackWeightHardening = new BooleanYoVariable("useFeedbackWeightHardening", registry);

   private final BooleanYoVariable scaleStepRegularizationWeightWithTime = new BooleanYoVariable("scaleStepRegularizationWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable("scaleFeedbackWeightWithGain", registry);
   private final BooleanYoVariable scaleUpcomingStepWeights = new BooleanYoVariable("scaleUpcomingStepWeights", registry);

   private final BooleanYoVariable swingSpeedUpEnabled = new BooleanYoVariable(yoNamePrefix + "SwingSpeedUpEnabled", registry);

   private final BooleanYoVariable isStanding = new BooleanYoVariable(yoNamePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable(yoNamePrefix + "IsInTransfer", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(yoNamePrefix + "IsInitialTransfer", registry);

   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "DoubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(yoNamePrefix + "SingleSupportDuration", registry);
   private final DoubleYoVariable initialDoubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "InitialTransferDuration", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(yoNamePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   private final DoubleYoVariable initialTime = new DoubleYoVariable(yoNamePrefix + "InitialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(yoNamePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable timeRemainingInState = new DoubleYoVariable(yoNamePrefix + "TimeRemainingInState", registry);
   private final DoubleYoVariable speedUpTime = new DoubleYoVariable(yoNamePrefix + "SpeedUpTime", registry);
   private final DoubleYoVariable minimumTimeRemaining = new DoubleYoVariable(yoNamePrefix + "MinimumTimeRemaining", registry);

   private final FramePoint2d currentICP = new FramePoint2d();
   private final FramePoint2d desiredICP = new FramePoint2d();
   private final FrameVector2d desiredICPVelocity = new FrameVector2d();
   private final FramePoint2d referenceCMP = new FramePoint2d();

   private final YoFramePoint2d controllerFeedbackCMP = new YoFramePoint2d(yoNamePrefix + "FeedbackCMP", worldFrame, registry);
   private final YoFrameVector2d controllerFeedbackCMPDelta = new YoFrameVector2d(yoNamePrefix + "FeedbackCMPDelta", worldFrame, registry);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);

   private final FramePoint2d finalICPRecursion = new FramePoint2d();
   private final FramePoint2d cmpOffsetRecursionEffect = new FramePoint2d();
   private final FramePoint2d stanceCMPProjection = new FramePoint2d();
   private final FramePoint2d beginningOfStateICPProjection = new FramePoint2d();

   private final YoFramePoint2d beginningOfStateICP = new YoFramePoint2d(yoNamePrefix + "BeginningOfStateICP", worldFrame, registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<YoFramePoint2d> upcomingFootstepLocations = new ArrayList<>();
   private final ArrayList<YoFramePoint2d> footstepSolutions = new ArrayList<>();

   private final DoubleYoVariable forwardFootstepWeight = new DoubleYoVariable(yoNamePrefix + "ForwardFootstepWeight", registry);
   private final DoubleYoVariable lateralFootstepWeight = new DoubleYoVariable(yoNamePrefix + "LateralFootstepWeight", registry);
   private final YoFramePoint2d scaledFootstepWeights = new YoFramePoint2d(yoNamePrefix + "ScaledFootstepWeights", worldFrame, registry);

   private final DoubleYoVariable feedbackForwardWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackForwardWeight", registry);
   private final DoubleYoVariable feedbackLateralWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackLateralWeight", registry);
   private final YoFramePoint2d scaledFeedbackWeight = new YoFramePoint2d(yoNamePrefix + "ScaledFeedbackWeight", worldFrame, registry);

   private final DoubleYoVariable footstepRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "FootstepRegularizationWeight", registry);
   private final DoubleYoVariable feedbackRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackRegularizationWeight", registry);
   private final DoubleYoVariable scaledFootstepRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "ScaledFootstepRegularizationWeight", registry);
   private final DoubleYoVariable dynamicRelaxationWeight = new DoubleYoVariable(yoNamePrefix + "DynamicRelaxationWeight", registry);

   private final DoubleYoVariable feedbackOrthogonalGain = new DoubleYoVariable(yoNamePrefix + "FeedbackOrthogonalGain", registry);
   private final DoubleYoVariable feedbackParallelGain = new DoubleYoVariable(yoNamePrefix + "FeedbackParallelGain", registry);
   private final YoFramePoint2d yoFeedbackGains = new YoFramePoint2d(yoNamePrefix + "EquivalentFeedbackGains", worldFrame, registry);

   private final DoubleYoVariable remainingTimeToStopAdjusting = new DoubleYoVariable(yoNamePrefix + "RemainingTimeToStopAdjusting", registry);

   private final IntegerYoVariable numberOfIterations = new IntegerYoVariable(yoNamePrefix + "NumberOfIterations", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable(yoNamePrefix + "HasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable(yoNamePrefix + "HasNotConvergedCounts", registry);

   private final DoubleYoVariable upcomingDoubleSupportSplitFraction;
   private final DoubleYoVariable defaultDoubleSupportSplitFraction;
   private final DoubleYoVariable doubleSupportSplitFractionUnderDisturbance;

   private final ICPOptimizationSolver solver;
   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator;

   private final ICPOptimizationCMPConstraintHandler cmpConstraintHandler;
   private final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   private final ICPOptimizationSolutionHandler solutionHandler;
   private final ICPOptimizationInputHandler inputHandler;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final double controlDT;
   private final double dynamicRelaxationDoubleSupportWeightModifier;
   private final int maximumNumberOfFootstepsToConsider;

   private boolean localUseTwoCMPs;
   private boolean localUseInitialICP;
   private boolean localUseFeedback;
   private boolean localUseFeedbackRegularization;
   private boolean localUseFeedbackWeightHardening;
   private boolean localUseStepAdjustment;
   private boolean localUseFootstepRegularization;

   private boolean localScaleUpcomingStepWeights;

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   public ICPOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.controlDT = controlDT;

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      initialDoubleSupportDuration.set(icpPlannerParameters.getDoubleSupportInitialTransferDuration());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      solver = new ICPOptimizationSolver(icpOptimizationParameters, totalVertices, registry);

      useTwoCMPsInControl.set(icpPlannerParameters.useTwoCMPsPerSupport());

      useInitialICP.set(icpOptimizationParameters.useICPFromBeginningOfState());
      useFeedback.set(icpOptimizationParameters.useFeedback());
      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());
      useFootstepRegularization.set(icpOptimizationParameters.useFootstepRegularization());
      useFeedbackRegularization.set(icpOptimizationParameters.useFeedbackRegularization());
      useFeedbackWeightHardening.set(icpOptimizationParameters.useFeedbackWeightHardening());

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

      minimumTimeRemaining.set(icpOptimizationParameters.getMinimumTimeRemaining());

      remainingTimeToStopAdjusting.set(icpOptimizationParameters.getRemainingTimeToStopAdjusting());
      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());
      else
         swingSpeedUpEnabled.set(false);

      dynamicRelaxationDoubleSupportWeightModifier = icpOptimizationParameters.getDynamicRelaxationDoubleSupportWeightModifier();

      DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(yoNamePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);
      defaultDoubleSupportSplitFraction = new DoubleYoVariable(yoNamePrefix + "DefaultDoubleSupportSplitFraction", registry);
      doubleSupportSplitFractionUnderDisturbance = new DoubleYoVariable(yoNamePrefix + "DoubleSupportSplitFractionUnderDisturbance", registry);
      upcomingDoubleSupportSplitFraction = new DoubleYoVariable(yoNamePrefix + "UpcomingDoubleSupportSplitFraction", registry);

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      defaultDoubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());
      doubleSupportSplitFractionUnderDisturbance.set(icpOptimizationParameters.getDoubleSupportSplitFractionUnderDisturbance());
      upcomingDoubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());

      footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(icpPlannerParameters, exitCMPDurationInPercentOfStepTime,
            defaultDoubleSupportSplitFraction, upcomingDoubleSupportSplitFraction, maximumNumberOfFootstepsToConsider, registry);

      cmpConstraintHandler = new ICPOptimizationCMPConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, registry);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, registry);
      solutionHandler = new ICPOptimizationSolutionHandler(icpOptimizationParameters, footstepRecursionMultiplierCalculator, VISUALIZE, registry, yoGraphicsListRegistry);
      inputHandler = new ICPOptimizationInputHandler(icpPlannerParameters, bipedSupportPolygons, contactableFeet, maximumNumberOfFootstepsToConsider,
            footstepRecursionMultiplierCalculator, doubleSupportDuration, singleSupportDuration, exitCMPDurationInPercentOfStepTime, upcomingDoubleSupportSplitFraction,
            VISUALIZE, registry, yoGraphicsListRegistry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         upcomingFootstepLocations.add(new YoFramePoint2d("upcomingFootstepLocation" + i, worldFrame, registry));
         footstepSolutions.add(new YoFramePoint2d("footstepSolutionLocation" + i, worldFrame, registry));
      }

      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

      parentRegistry.addChild(registry);
   }

   public void setStepDurations(double doubleSupportDuration, double singleSupportDuration)
   {
      setDoubleSupportDuration(doubleSupportDuration);
      setSingleSupportDuration(singleSupportDuration);
   }

   public void setDoubleSupportDuration(double doubleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration);
   }

   public void setSingleSupportDuration(double singleSupportDuration)
   {
      this.singleSupportDuration.set(singleSupportDuration);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      footstepRecursionMultiplierCalculator.reset();
      inputHandler.clearPlan();
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         upcomingFootstepLocations.get(i).setToZero();
   }

   private final FramePoint2d tmpFramePoint2d = new FramePoint2d();
   public void addFootstepToPlan(Footstep footstep)
   {
      if (footstep != null)
      {
         upcomingFootsteps.add(footstep);
         footstep.getPosition2d(tmpFramePoint2d);
         upcomingFootstepLocations.get(upcomingFootsteps.size() - 1).set(tmpFramePoint2d);
         inputHandler.addFootstepToPlan(footstep);

         footstepSolutions.get(upcomingFootsteps.size() - 1).set(tmpFramePoint2d);
      }
   }

   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);
      isInitialTransfer.set(true);

      setProblemBooleans();

      footstepRecursionMultiplierCalculator.resetTimes();

      cmpConstraintHandler.updateCMPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);

      speedUpTime.set(0.0);
   }

   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0)
   {
      this.initialTime.set(initialTime);
      this.transferToSide.set(transferToSide);
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      isInTransfer.set(true);

      beginningOfStateICP.set(solutionHandler.getControllerReferenceICP());

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      setProblemBooleans();

      footstepRecursionMultiplierCalculator.resetTimes();
      if (isInitialTransfer.getBooleanValue())
         footstepRecursionMultiplierCalculator.submitTimes(0, initialDoubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      else
         footstepRecursionMultiplierCalculator.submitTimes(0, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, isInTransfer.getBooleanValue(),
            localUseTwoCMPs, omega0);

      inputHandler.initializeForDoubleSupport(isStanding.getBooleanValue(), localUseTwoCMPs, transferToSide, omega0);

      if (localUseFootstepRegularization)
         resetFootstepRegularizationTask();
      if (localUseFeedbackRegularization)
         solver.resetFeedbackRegularization();

      cmpConstraintHandler.updateCMPConstraintForDoubleSupport(solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);

      speedUpTime.set(0.0);
   }

   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0)
   {
      this.initialTime.set(initialTime);
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInTransfer.set(false);
      isInitialTransfer.set(false);

      upcomingDoubleSupportSplitFraction.set(defaultDoubleSupportSplitFraction.getDoubleValue());

      beginningOfStateICP.set(solutionHandler.getControllerReferenceICP());

      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      setProblemBooleans();

      footstepRecursionMultiplierCalculator.resetTimes();
      footstepRecursionMultiplierCalculator.submitTimes(0, 0.0, singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      footstepRecursionMultiplierCalculator.submitTimes(numberOfFootstepsToConsider + 1, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, isInTransfer.getBooleanValue(),
            localUseTwoCMPs, omega0);

      inputHandler.initializeForSingleSupport(localUseTwoCMPs, supportSide, omega0);

      if (localUseFootstepRegularization)
         resetFootstepRegularizationTask();
      if (localUseFeedbackRegularization)
         solver.resetFeedbackRegularization();

      cmpConstraintHandler.updateCMPConstraintForSingleSupport(supportSide, solver);
      reachabilityConstraintHandler.updateReachabilityConstraintForSingleSupport(supportSide, solver);

      speedUpTime.set(0.0);
   }

   private void setProblemBooleans()
   {
      localUseTwoCMPs = useTwoCMPsInControl.getBooleanValue();
      localUseInitialICP = useInitialICP.getBooleanValue();
      localUseFeedback = useFeedback.getBooleanValue();
      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localUseFootstepRegularization = useFootstepRegularization.getBooleanValue();
      localUseFeedbackRegularization = useFeedbackRegularization.getBooleanValue();
      localUseFeedbackWeightHardening = useFeedbackWeightHardening.getBooleanValue();

      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();
   }

   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FrameVector2d desiredCMPDelta = new FrameVector2d();

   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP, double omega0)
   {
      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      this.currentICP.set(currentICP);
      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);

      solutionHandler.getControllerReferenceCMP(referenceCMP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      int numberOfFootstepsToConsider = checkForEndingOfAdjustment(omega0);

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      if (isStanding.getBooleanValue())
         setConditionsForFeedbackOnlyControl();
      else
         setConditionsForSteppingControl(numberOfFootstepsToConsider, omega0);

      NoConvergenceException noConvergenceException = null;
      try
      {
         solver.compute(finalICPRecursion, cmpOffsetRecursionEffect, currentICP, referenceCMP, stanceCMPProjection, beginningOfStateICPProjection);
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

      // don't pole the new solutions if there's a no convergence exception
      if (noConvergenceException == null)
      {
         numberOfIterations.set(solver.getNumberOfIterations());

         if (localUseStepAdjustment)
            solutionHandler.extractFootstepSolutions(footstepSolutions, upcomingFootstepLocations, upcomingFootsteps, numberOfFootstepsToConsider, solver);

         solver.getCMPFeedbackDifference(desiredCMPDelta);
         solutionHandler.updateCostsToGo(solver);
      }

      if (isStanding.getBooleanValue())
      {
         solutionHandler.setValuesForFeedbackOnly(desiredICP, desiredICPVelocity, omega0);
      }
      else
      {
         solutionHandler.computeReferenceFromSolutions(footstepSolutions, inputHandler, beginningOfStateICP, omega0, numberOfFootstepsToConsider);
         solutionHandler.computeNominalValues(upcomingFootstepLocations, inputHandler, beginningOfStateICP, omega0, numberOfFootstepsToConsider);

         if (solutionHandler.wasFootstepAdjusted())
            upcomingDoubleSupportSplitFraction.set(doubleSupportSplitFractionUnderDisturbance.getDoubleValue());
         else
            upcomingDoubleSupportSplitFraction.set(defaultDoubleSupportSplitFraction.getDoubleValue());
      }

      solutionHandler.getControllerReferenceCMP(desiredCMP);

      icpError.set(currentICP);
      icpError.sub(solutionHandler.getControllerReferenceICP());

      desiredCMP.add(desiredCMPDelta);

      controllerFeedbackCMP.set(desiredCMP);
      controllerFeedbackCMPDelta.set(desiredCMPDelta);
   }

   private void setConditionsForFeedbackOnlyControl()
   {
      solver.submitProblemConditions(0, false, true, false);

      setFeedbackConditions();

      finalICPRecursion.set(desiredICP);
      cmpOffsetRecursionEffect.setToZero();
      stanceCMPProjection.setToZero();
      beginningOfStateICPProjection.setToZero();
   }

   private int setConditionsForSteppingControl(int numberOfFootstepsToConsider, double omega0)
   {
      if (isInTransfer.getBooleanValue())
      {
         cmpConstraintHandler.updateCMPConstraintForDoubleSupport(solver);
         reachabilityConstraintHandler.updateReachabilityConstraintForDoubleSupport(solver);
      }
      else
      {
         cmpConstraintHandler.updateCMPConstraintForSingleSupport(supportSide.getEnumValue(), solver);
         reachabilityConstraintHandler.updateReachabilityConstraintForSingleSupport(supportSide.getEnumValue(), solver);
      }

      solver.submitProblemConditions(numberOfFootstepsToConsider, localUseStepAdjustment, localUseFeedback, localUseTwoCMPs);

      if (localUseFeedback)
      {
         setFeedbackConditions();

         if (localUseFeedbackWeightHardening)
            solver.setUseFeedbackWeightHardening();

         if (localUseFeedbackRegularization)
            solver.setFeedbackRegularizationWeight(feedbackRegularizationWeight.getDoubleValue() / controlDT);
      }

      if (localUseStepAdjustment && !isInTransfer.getBooleanValue())
      {
         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
            submitFootstepConditionsToSolver(footstepIndex);

         if (localUseFootstepRegularization)
            solver.setFootstepRegularizationWeight(scaledFootstepRegularizationWeight.getDoubleValue() / controlDT);
      }

      double clippedTimeRemaining = Math.max(minimumTimeRemaining.getDoubleValue(), timeRemainingInState.getDoubleValue());
      inputHandler.update(localUseTwoCMPs, omega0);
      inputHandler.computeFinalICPRecursion(finalICPRecursion, numberOfFootstepsToConsider, localUseTwoCMPs, isInTransfer.getBooleanValue(), omega0);
      inputHandler.computeStanceCMPProjection(stanceCMPProjection, clippedTimeRemaining, localUseTwoCMPs, isInTransfer.getBooleanValue(), localUseInitialICP, omega0);
      inputHandler.computeBeginningOfStateICPProjection(beginningOfStateICPProjection, beginningOfStateICP.getFrameTuple2d());

      inputHandler.computeStanceCMPProjection(stanceCMPProjection, clippedTimeRemaining, localUseTwoCMPs, isInTransfer.getBooleanValue(), localUseInitialICP, omega0);

      inputHandler.computeCMPOffsetRecursionEffect(cmpOffsetRecursionEffect, upcomingFootstepLocations, numberOfFootstepsToConsider);
      if (!localUseTwoCMPs)
      {
         cmpOffsetRecursionEffect.setToZero();
      }

      return numberOfFootstepsToConsider;
   }

   private final FrameVector2d feedbackGains = new FrameVector2d();
   private void setFeedbackConditions()
   {
      getTransformedFeedbackGains(feedbackGains);

      double dynamicRelaxationWeight = this.dynamicRelaxationWeight.getDoubleValue();
      if (!localUseStepAdjustment)
         dynamicRelaxationWeight = dynamicRelaxationWeight / dynamicRelaxationDoubleSupportWeightModifier;

      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), feedbackGains.getX(), feedbackGains.getY(), dynamicRelaxationWeight);
   }

   private final RigidBodyTransform transformTranspose = new RigidBodyTransform();
   private final Matrix3d rotation = new Matrix3d();
   private final Matrix3d rotationTranspose = new Matrix3d();
   private final Matrix3d gainsMatrix = new Matrix3d();
   private final Matrix3d gainsMatrixTransformed = new Matrix3d();

   private void getTransformedFeedbackGains(FrameVector2d feedbackGainsToPack)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         RigidBodyTransform transform = icpVelocityDirectionFrame.getTransformToWorldFrame();

         transform.getRotation(rotation);
         rotationTranspose.set(rotation);
         rotationTranspose.transpose();
         transformTranspose.setRotation(rotationTranspose);

         gainsMatrix.setZero();
         gainsMatrix.setElement(0, 0, 1.0 + feedbackParallelGain.getDoubleValue());
         gainsMatrix.setElement(1, 1, 1.0 + feedbackOrthogonalGain.getDoubleValue());

         gainsMatrixTransformed.set(rotation);
         gainsMatrixTransformed.mul(gainsMatrix);
         gainsMatrixTransformed.mul(rotationTranspose);

         feedbackGainsToPack.setToZero(worldFrame);
         feedbackGainsToPack.setX(gainsMatrixTransformed.getElement(0, 0));
         feedbackGainsToPack.setY(gainsMatrixTransformed.getElement(1, 1));
      }
      else
      {
         feedbackGainsToPack.setToZero(worldFrame);
         feedbackGainsToPack.set(feedbackOrthogonalGain.getDoubleValue(), feedbackOrthogonalGain.getDoubleValue());
      }
      yoFeedbackGains.set(feedbackGainsToPack);
   }

   private final Matrix3d weightsMatrix = new Matrix3d();
   private final Matrix3d weightsMatrixTransformed = new Matrix3d();

   private void getTransformedWeights(FrameVector2d weightsToPack, double forwardWeight, double lateralWeight)
   {
      RigidBodyTransform transform = contactableFeet.get(supportSide.getEnumValue()).getSoleFrame().getTransformToWorldFrame();

      transform.getRotation(rotation);
      rotationTranspose.set(rotation);
      rotation.transpose();
      transformTranspose.setRotation(rotationTranspose);

      weightsMatrix.setZero();
      weightsMatrix.setElement(0, 0, forwardWeight);
      weightsMatrix.setElement(1, 1, lateralWeight);

      weightsMatrixTransformed.set(rotation);
      weightsMatrixTransformed.mul(weightsMatrix);
      weightsMatrixTransformed.mul(rotationTranspose);

      weightsToPack.setToZero(worldFrame);
      weightsToPack.setX(weightsMatrixTransformed.getElement(0, 0));
      weightsToPack.setY(weightsMatrixTransformed.getElement(1, 1));
   }

   private final FramePose footstepPose = new FramePose();
   private int checkForEndingOfAdjustment(double omega0)
   {
      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      if (timeRemainingInState.getDoubleValue() < remainingTimeToStopAdjusting.getDoubleValue() && localUseStepAdjustment && !isStanding.getBooleanValue() && !isInTransfer.getBooleanValue())
      {
         //record current locations
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            upcomingFootstepLocations.get(i).set(footstepSolutions.get(i));

            Footstep footstep = upcomingFootsteps.get(i);
            footstep.getPose(footstepPose);
            footstepPose.setXYFromPosition2d(footstepSolutions.get(i).getFrameTuple2d());
            footstep.setPose(footstepPose);

            inputHandler.addFootstepToPlan(footstep);
         }

         localUseStepAdjustment = false;

         numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

         footstepRecursionMultiplierCalculator.resetTimes();
         footstepRecursionMultiplierCalculator.submitTimes(0, 0.0, singleSupportDuration.getDoubleValue());

         for (int i = 1; i < numberOfFootstepsToConsider + 1; i++)
            footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
         footstepRecursionMultiplierCalculator.submitTimes(numberOfFootstepsToConsider + 1, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

         footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, isInTransfer.getBooleanValue(), localUseTwoCMPs, omega0);
      }

      return numberOfFootstepsToConsider;
   }

   private void resetFootstepRegularizationTask()
   {
      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         solver.resetFootstepRegularization(i, upcomingFootstepLocations.get(i).getFrameTuple2d());
   }

   private int clipNumberOfFootstepsToConsiderToProblem(int numberOfFootstepsToConsider)
   {
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, upcomingFootsteps.size());
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

      if (!localUseStepAdjustment || isInTransfer.getBooleanValue() || isStanding.getBooleanValue())
         numberOfFootstepsToConsider = 0;

      return numberOfFootstepsToConsider;
   }

   private final FrameVector2d footstepWeights = new FrameVector2d();
   private void submitFootstepConditionsToSolver(int footstepIndex)
   {
      getTransformedWeights(footstepWeights, forwardFootstepWeight.getDoubleValue(), lateralFootstepWeight.getDoubleValue());
      scaledFootstepWeights.set(footstepWeights);

      if (localScaleUpcomingStepWeights)
         scaledFootstepWeights.scale(1.0 / (footstepIndex + 1));

      double footstepRecursionMultiplier;
      if (localUseTwoCMPs)
      {
         double entryMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(footstepIndex);
         double exitMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);

         footstepRecursionMultiplier = entryMutliplier + exitMutliplier;
      }
      else
      {
         footstepRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);
      }
      footstepRecursionMultiplier *= footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier();

      solver.setFootstepAdjustmentConditions(footstepIndex, footstepRecursionMultiplier, scaledFootstepWeights.getX(), scaledFootstepWeights.getY(),
            upcomingFootstepLocations.get(footstepIndex).getFrameTuple2d());
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
         double remainingTime;
         if (isInTransfer.getBooleanValue())
            remainingTime = doubleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
         else
            remainingTime = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();

         timeRemainingInState.set(remainingTime);
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


   private void scaleStepRegularizationWeightWithTime()
   {
      if (scaleStepRegularizationWeightWithTime.getBooleanValue())
      {
         double alpha = Math.max(timeRemainingInState.getDoubleValue(), minimumTimeRemaining.getDoubleValue()) / singleSupportDuration.getDoubleValue();
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue() / alpha);
      }
      else
      {
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue());
      }
   }

   private final FrameVector2d feedbackWeights = new FrameVector2d();
   private void scaleFeedbackWeightWithGain()
   {
      getTransformedWeights(feedbackWeights, feedbackForwardWeight.getDoubleValue(), feedbackLateralWeight.getDoubleValue());
      scaledFeedbackWeight.set(feedbackWeights);

      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         getTransformedFeedbackGains(feedbackGains);

         double alpha = Math.sqrt(Math.pow(feedbackGains.getX(), 2) + Math.pow(feedbackGains.getY(), 2));
         scaledFeedbackWeight.scale(1.0 / alpha);
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

   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3d x = new Vector3d();
      private final Vector3d y = new Vector3d();
      private final Vector3d z = new Vector3d();
      private final Matrix3d rotation = new Matrix3d();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumn(0, x);
         rotation.setColumn(1, y);
         rotation.setColumn(2, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
