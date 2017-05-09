package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

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
