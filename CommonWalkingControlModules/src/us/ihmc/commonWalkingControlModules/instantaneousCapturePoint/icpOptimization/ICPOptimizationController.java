package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.StateMultiplierCalculator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public abstract class ICPOptimizationController
{
   protected static final boolean VISUALIZE = true;
   protected static final boolean COMPUTE_COST_TO_GO = false;
   protected static final boolean ALLOW_ADJUSTMENT_IN_TRANSFER = false;
   protected static final boolean DEBUG = false;

   protected static final String yoNamePrefix = "controller";

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable(yoNamePrefix + "NumberOfFootstepsToConsider", registry);

   protected final BooleanYoVariable useStepAdjustment = new BooleanYoVariable(yoNamePrefix + "UseStepAdjustment", registry);
   protected final BooleanYoVariable useAngularMomentum = new BooleanYoVariable(yoNamePrefix + "UseAngularMomentum", registry);

   protected final BooleanYoVariable scaleStepRegularizationWeightWithTime = new BooleanYoVariable(yoNamePrefix + "ScaleStepRegularizationWeightWithTime", registry);
   protected final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable(yoNamePrefix + "ScaleFeedbackWeightWithGain", registry);
   protected final BooleanYoVariable scaleUpcomingStepWeights = new BooleanYoVariable(yoNamePrefix + "ScaleUpcomingStepWeights", registry);

   protected final BooleanYoVariable isStanding = new BooleanYoVariable(yoNamePrefix + "IsStanding", registry);
   protected final BooleanYoVariable isInTransfer = new BooleanYoVariable(yoNamePrefix + "IsInTransfer", registry);

   protected final List<DoubleYoVariable> swingDurations = new ArrayList<>();
   protected final List<DoubleYoVariable> transferDurations = new ArrayList<>();
   protected final DoubleYoVariable finalTransferDuration = new DoubleYoVariable(yoNamePrefix + "FinalTransferDuration", registry);

   protected final List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
   protected final List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
   protected final DoubleYoVariable defaultTransferSplitFraction = new DoubleYoVariable(yoNamePrefix + "DefaultTransferSplitFraction", registry);
   protected final DoubleYoVariable defaultSwingSplitFraction = new DoubleYoVariable(yoNamePrefix + "DefaultSwingSplitFraction", registry);

   protected final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(yoNamePrefix + "TransferToSide", registry, RobotSide.class, true);
   protected final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(yoNamePrefix + "SupportSide", registry, RobotSide.class, true);

   protected final DoubleYoVariable initialTime = new DoubleYoVariable(yoNamePrefix + "InitialTime", registry);
   protected final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(yoNamePrefix + "TimeInCurrentState", registry);
   protected final DoubleYoVariable timeRemainingInState = new DoubleYoVariable(yoNamePrefix + "TimeRemainingInState", registry);
   protected final DoubleYoVariable minimumTimeRemaining = new DoubleYoVariable(yoNamePrefix + "MinimumTimeRemaining", registry);

   protected final YoFrameVector2d icpError = new YoFrameVector2d(yoNamePrefix + "IcpError", "", worldFrame, registry);
   protected final YoFramePoint2d controllerFeedbackCMP = new YoFramePoint2d(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   protected final YoFrameVector2d controllerFeedbackCMPDelta = new YoFrameVector2d(yoNamePrefix + "FeedbackCMPDeltaSolution", worldFrame, registry);
   protected final YoFramePoint2d dynamicRelaxation = new YoFramePoint2d(yoNamePrefix + "DynamicRelaxationSolution", "", worldFrame, registry);
   protected final YoFramePoint2d cmpCoPDifferenceSolution = new YoFramePoint2d(yoNamePrefix + "CMPCoPDifferenceSolution", "", worldFrame, registry);

   protected final YoFramePoint2d beginningOfStateICP = new YoFramePoint2d(yoNamePrefix + "BeginningOfStateICP", worldFrame, registry);
   protected final YoFrameVector2d beginningOfStateICPVelocity = new YoFrameVector2d(yoNamePrefix + "BeginningOfStateICPVelocity", worldFrame, registry);

   protected final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   protected final ArrayList<YoFramePoint2d> upcomingFootstepLocations = new ArrayList<>();
   protected final ArrayList<YoFramePoint2d> footstepSolutions = new ArrayList<>();
   protected final ArrayList<FramePoint2d> unclippedFootstepSolutions = new ArrayList<>();

   protected final DoubleYoVariable forwardFootstepWeight = new DoubleYoVariable(yoNamePrefix + "ForwardFootstepWeight", registry);
   protected final DoubleYoVariable lateralFootstepWeight = new DoubleYoVariable(yoNamePrefix + "LateralFootstepWeight", registry);
   protected final YoFramePoint2d scaledFootstepWeights = new YoFramePoint2d(yoNamePrefix + "ScaledFootstepWeights", worldFrame, registry);

   protected final DoubleYoVariable feedbackForwardWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackForwardWeight", registry);
   protected final DoubleYoVariable feedbackLateralWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackLateralWeight", registry);
   protected final YoFramePoint2d scaledFeedbackWeight = new YoFramePoint2d(yoNamePrefix + "ScaledFeedbackWeight", worldFrame, registry);

   protected final DoubleYoVariable footstepRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "FootstepRegularizationWeight", registry);
   protected final DoubleYoVariable feedbackRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "FeedbackRegularizationWeight", registry);
   protected final DoubleYoVariable scaledFootstepRegularizationWeight = new DoubleYoVariable(yoNamePrefix + "ScaledFootstepRegularizationWeight", registry);
   protected final DoubleYoVariable dynamicRelaxationWeight = new DoubleYoVariable(yoNamePrefix + "DynamicRelaxationWeight", registry);
   protected final DoubleYoVariable angularMomentumMinimizationWeight = new DoubleYoVariable(yoNamePrefix + "AngularMomentumMinimizationWeight", registry);

   protected final DoubleYoVariable feedbackOrthogonalGain = new DoubleYoVariable(yoNamePrefix + "FeedbackOrthogonalGain", registry);
   protected final DoubleYoVariable feedbackParallelGain = new DoubleYoVariable(yoNamePrefix + "FeedbackParallelGain", registry);

   protected final IntegerYoVariable numberOfIterations = new IntegerYoVariable(yoNamePrefix + "NumberOfIterations", registry);
   protected final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable(yoNamePrefix + "HasNotConvergedInPast", registry);
   protected final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable(yoNamePrefix + "HasNotConvergedCounts", registry);

   protected final BooleanYoVariable doingBigAdjustment = new BooleanYoVariable(yoNamePrefix + "DoingBigAdjustment", registry);

   protected final DoubleYoVariable transferSplitFractionUnderDisturbance = new DoubleYoVariable(yoNamePrefix + "DoubleSupportSplitFractionUnderDisturbance", registry);
   protected final DoubleYoVariable magnitudeForBigAdjustment = new DoubleYoVariable(yoNamePrefix + "MagnitudeForBigAdjustment", registry);

   protected final FramePoint2d currentICP = new FramePoint2d();
   protected final FramePoint2d desiredICP = new FramePoint2d();
   protected final FrameVector2d desiredICPVelocity = new FrameVector2d();
   protected final FramePoint2d referenceCMP = new FramePoint2d();

   protected final FramePoint2d finalICPRecursion = new FramePoint2d();
   protected final FramePoint2d cmpConstantEffects = new FramePoint2d();

   protected final StateMultiplierCalculator stateMultiplierCalculator;

   protected final ICPOptimizationCoPConstraintHandler copConstraintHandler;
   protected final ICPOptimizationReachabilityConstraintHandler reachabilityConstraintHandler;
   protected final ICPOptimizationSolutionHandler solutionHandler;
   protected final ICPOptimizationInputHandler inputHandler;

   protected final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   protected final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();

   protected boolean localUseStepAdjustment;
   protected boolean localScaleUpcomingStepWeights;

   protected final FrameVector2d tempVector2d = new FrameVector2d();
   protected final FramePoint2d tempPoint2d = new FramePoint2d();

   protected final double controlDT;
   protected final double dynamicRelaxationDoubleSupportWeightModifier;
   protected final int maximumNumberOfFootstepsToConsider;

   protected final boolean useDifferentSplitRatioForBigAdjustment;
   protected final double minimumTimeOnInitialCMPForBigAdjustment;

   public ICPOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         WalkingControllerParameters walkingControllerParameters, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.controlDT = controlDT;

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getRigidBody().getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      dynamicRelaxationDoubleSupportWeightModifier = icpOptimizationParameters.getDynamicRelaxationDoubleSupportWeightModifier();

      stateMultiplierCalculator = new StateMultiplierCalculator(icpPlannerParameters, transferDurations, swingDurations, transferSplitFractions,
            swingSplitFractions, maximumNumberOfFootstepsToConsider, yoNamePrefix, registry);

      copConstraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons);
      reachabilityConstraintHandler = new ICPOptimizationReachabilityConstraintHandler(bipedSupportPolygons, icpOptimizationParameters, yoNamePrefix, registry);
      solutionHandler = new ICPOptimizationSolutionHandler(icpOptimizationParameters, transformsFromAnkleToSole, VISUALIZE, DEBUG, yoNamePrefix, registry,
            yoGraphicsListRegistry);
      inputHandler = new ICPOptimizationInputHandler(icpPlannerParameters, bipedSupportPolygons, contactableFeet, maximumNumberOfFootstepsToConsider,
            transferDurations, swingDurations, transferSplitFractions, swingSplitFractions, VISUALIZE, yoNamePrefix, registry, yoGraphicsListRegistry);

      useDifferentSplitRatioForBigAdjustment = icpOptimizationParameters.useDifferentSplitRatioForBigAdjustment();
      minimumTimeOnInitialCMPForBigAdjustment = icpOptimizationParameters.getMinimumTimeOnInitialCMPForBigAdjustment();
   }

   /**
    * Sets the weight on tracking the reference footstep locations in the optimization.
    *
    * @param forwardWeight tracking weight in the forward direction.
    * @param lateralWeight tracking weight in the lateral direction.
    */
   public abstract void setFootstepWeights(double forwardWeight, double lateralWeight);

   /**
    * Sets the weight on minimizing the feedback action for the optimization.
    *
    * @param forwardWeight feedback minimization weight in the forward direction.
    * @param lateralWeight feedback minimization weight in the lateral direction.
    */
   public abstract void setFeedbackWeights(double forwardWeight, double lateralWeight);

   /**
    * Clear footstep and timing information making the ICP planner ready to be reinitialized with
    * new footsteps.
    * <p>
    * Don't forget to call this method before registering a new set of footsteps.
    * </p>
    */
   public abstract void clearPlan();

   public abstract void setTransferDuration(int stepNumber, double duration);

   public abstract void setSwingDuration(int stepNumber, double duration);

   /**
    * Allows setting of the transfer duration split fraction (see {@link #transferSplitFractions}) for the specified step number.
    *
    * @param stepNumber step transfer duration split fraction to modify.
    * @param splitFraction new transfer duration split fraction value.
    */
   public abstract void setTransferSplitFraction(int stepNumber, double splitFraction);

   /**
    * Allows setting of the swing duration split fraction (see {@link #swingSplitFractions}) for the specified step number.
    *
    * @param stepNumber step swing duration split fraction to modify.
    * @param splitFraction new swing duration split fraction value.
    */
   public abstract void setSwingSplitFraction(int stepNumber, double splitFraction);

   /**
    * Changes the duration for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    *
    * @param finalTransferDuration final transfer duration
    */
   public abstract void setFinalTransferDuration(double finalTransferDuration);

   /**
    * Changes the split fraction for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    *
    * @param splitFraction final transfer duration
    */
   public abstract void setFinalTransferSplitFraction(double splitFraction);

   public abstract void setFinalTransferSplitFractionToDefault();

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
   public abstract void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   public abstract void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

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
   public abstract void initializeForStanding(double initialTime);

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
   public abstract void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0);

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
   public abstract void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0);

   public abstract void setBeginningOfStateICP(FramePoint2d beginningOfStateICP, FrameVector2d beginningOfStateICPVelocity);







   public abstract void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP, double omega0);

   public abstract int getNumberOfFootstepsToConsider();

   public abstract void getDesiredCMP(FramePoint2d desiredCMPToPack);

   public abstract void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack);

   public abstract boolean wasFootstepAdjusted();

   public abstract boolean useAngularMomentum();
}
