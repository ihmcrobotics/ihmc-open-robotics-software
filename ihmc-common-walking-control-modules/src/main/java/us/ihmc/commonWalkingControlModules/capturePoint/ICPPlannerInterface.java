package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public interface ICPPlannerInterface
{

   /**
    * Clear footstep and timing information making the ICP planner ready to be reinitialized with
    * new footsteps.
    * <p>
    * Don't forget to call this method before registering a new set of footsteps.
    * </p>
    */
   void clearPlan();

   /**
    * Registers the side of the support leg.
    * <p>
    * This is required before initializing the planner for a single support phase.
    * </p>
    *
    * @param robotSide the side of the support leg.
    */
   void setSupportLeg(RobotSide robotSide);

   /**
    * Registers the side to which the robot is about to transfer its weight.
    * <p>
    * This is required before initializing the planner for a transfer phase.
    * </p>
    *
    * @param robotSide the side towards which the robot is about to transfer.
    */
   void setTransferToSide(RobotSide robotSide);

   /**
    * Registers the side from which the robot is about to transfer its weight.
    * <p>
    * This is equivalent to: {@code setTransferToSide(robotSide.getOppositeSide())}.
    * </p>
    * <p>
    * This is required before initializing the planner for a transfer phase.
    * </p>
    *
    * @param robotSide the side from which the robot is about to transfer.
    */
   void setTransferFromSide(RobotSide robotSide);

   /**
    * Registers an additional footstep to consider in the next plan.
    * <p>
    * Footsteps have to be registered before initializing the planner.
    * </p>
    * <p>
    * The reference to {@code footstep} is saved internally. The active ICP plan can be modified by
    * updating a footstep and then calling the method {@link #updateCurrentPlan()}.
    * </p>
    *
    * @param footstep the new footstep to be queued to the current list of footsteps. Not modified.
    * @param timing the timings to use when performing the footstep. Not modified.
    */
   void addFootstepToPlan(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions);

   /**
    * Prepares the planner to hold the given ICP position {@code icpPositionToHold} during the
    * double support phase.
    * <p>
    * This is usually useful for dealing with unexpected switch to double support where centering
    * the ICP in the support polygon would be undesirable.
    * </p>
    *
    * @param icpPositionToHold the position at which the ICP will be held during the next double
    *           support phase. Not modified.
    */
   void holdCurrentICP(FramePoint3D icpPositionToHold);

   /**
    * Initializes the planner to smoothly re-center the ICP in the support polygon preparing the
    * robot for standing.
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
   void initializeForStanding(double initialTime);

   /**
    * Prepares the ICP planner for a transfer phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the transfer side has been
    * registered using {@link #setTransferToSide(RobotSide)} before calling this method.
    * </p>
    *
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   void initializeForTransfer(double initialTime);

   /**
    * Computes the final CoM position in transfer. Uses most of the same methods as {@link #initializeForSingleSupport(double)},
    * but doesn't update the reference CMP calculator, making it slightly more computationally efficient
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the support side has been
    * registered using {@link #setSupportLeg(RobotSide)} before calling this method.
    * </p>
    */
   void computeFinalCoMPositionInTransfer();

   /**
    * Prepares the ICP planner for a single support phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the support side has been
    * registered using {@link #setSupportLeg(RobotSide)} before calling this method.
    * </p>
    *
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   void initializeForSingleSupport(double initialTime);

   /**
    * Computes the final CoM position in swing. Uses most of the same methods as {@link #initializeForSingleSupport(double)},
    * but doesn't update the reference CMP calculator, making it slightly more computationally efficient
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the support side has been
    * registered using {@link #setSupportLeg(RobotSide)} before calling this method.
    * </p>
    */
   void computeFinalCoMPositionInSwing();

   /**
    * Reinitializes the current plan without changing its initial time.
    * <p>
    * This is typically useful for updating the ICP plan with any change in contact state, i.e. on
    * foot switched to toe-off or the support polygon has been resized.
    * </p>
    * <p>
    * It can also be used to update the ICP plan when one of the registered footstep has been
    * modified from the outside. i.e. when dealing with push recovery via step adjustment.
    * </p>
    */
   void updateCurrentPlan();

   /**
    * Given the location of the actual ICP {@code actualCapturePointPosition}, this method estimates
    * the duration before the capture point reaches its desired location at foot touchdown.
    * <p>
    * Note this method is to be used when in single support and assumes that the internal state of
    * the planner is up-to-date, i.e. {@link #compute(double)} has been called in the current
    * control tick.
    *
    * @param actualCapturePointPosition the current position of the measured ICP. Not modified.
    * @return the estimated time remaining before the capture point reaches its desired position at
    *         the end of this state.
    */
   double estimateTimeRemainingForStateUnderDisturbance(FramePoint2DReadOnly actualCapturePointPosition);

   /**
    * Updates the current state of the ICP plan.
    * <p>
    * The ICP planner has to be initialized before calling this method.
    * </p>
    * <p>
    * The ICP planner has to be updated before accessing its outputs.
    * </p>
    *
    * @param time the current controller time.
    */
   void compute(double time);

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   void getDesiredCapturePointPosition(FramePoint3D desiredCapturePointPositionToPack);

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack);

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   void getDesiredCapturePointPosition(YoFramePoint3D desiredCapturePointPositionToPack);

   /**
    * Gets the current CoM position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfMassPositionToPack the current CoM position. Modified.
    */
   void getDesiredCenterOfMassPosition(FramePoint3D desiredCenterOfMassPositionToPack);

   /**
    * Gets the current CoM position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfMassPositionToPack the current CoM position. Modified.
    */
   void getDesiredCenterOfMassPosition(YoFramePoint3D desiredCenterOfMassPositionToPack);

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack);

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack);

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   void getDesiredCapturePointVelocity(YoFrameVector3D desiredCapturePointVelocityToPack);

   /**
    * Gets the current ICP acceleration.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointAccelerationToPack the current ICP velocity. Modified.
    */
   void getDesiredCapturePointAcceleration(FrameVector3D desiredCapturePointAccelerationToPack);

   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack);

   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack);

   /**
    * Gets the current CMP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotVelocityToPack the current CMP velocity. Modified.
    */
   void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack);

   /**
    * Gets the current CMP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotVelocityToPack the current CMP velocity. Modified.
    */
   void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack);

   /**
    * Gets the current CoP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfPressurePositionToPack the current CoP position. Modified.
    */
   void getDesiredCenterOfPressurePosition(FramePoint3D desiredCenterOfPressurePositionToPack);

   /**
    * Gets the current CoP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfPressurePositionToPack the current CoP position. Modified.
    */
   void getDesiredCenterOfPressurePosition(FramePoint2D desiredCenterOfPressurePositionToPack);

   /**
    * Gets the current CoP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfPressureVelocityToPack the current CoP velocity. Modified.
    */
   void getDesiredCenterOfPressureVelocity(FrameVector3D desiredCenterOfPressureVelocityToPack);

   /**
    * Gets the current CoP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfPressureVelocityToPack the current CoP velocity. Modified.
    */
   void getDesiredCenterOfPressureVelocity(FrameVector2D desiredCenterOfPressureVelocityToPack);

   /**
    * Gets the time relative to the beginning of the current state.
    *
    * @return the time spent in the current state.
    */
   double getTimeInCurrentState();

   /**
    * Gets the time remaining before the end of the current state.
    *
    * @return the time remaining.
    */
   double getTimeInCurrentStateRemaining();

   /**
    * Gets the current state overall duration.
    *
    * @return the current state duration.
    */
   double getCurrentStateDuration();

   void setTransferDuration(int stepNumber, double duration);

   void setSwingDuration(int stepNumber, double duration);

   double getTransferDuration(int stepNumber);

   double getSwingDuration(int stepNumber);

   /**
    * Changes the duration for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    *
    * @param duration
    */
   void setFinalTransferDuration(double duration);

   void setFinalTransferDurationAlpha(double durationAlpha);

   /**
    * Allows setting of the transfer duration alpha (see {@link #defaultTransferDurationAlpha}) for the specified step number.
    *
    * @param stepNumber step transfer duration alpha to modify.
    * @param transferDurationAlpha new transfer duration alpha value.
    */
   void setTransferDurationAlpha(int stepNumber, double transferDurationAlpha);

   /**
    * Allows setting of the swing duration alpha (see {@link #defaultSwingDurationAlpha}) for the specified step number.
    *
    * @param stepNumber step swing duration alpha to modify.
    * @param swingDurationAlpha new swing duration alpha value.
    */
   void setSwingDurationAlpha(int stepNumber, double swingDurationAlpha);

   double getTransferDurationAlpha(int stepNumber);

   double getSwingDurationAlpha(int stepNumber);

   /**
    * Gets the time at which the swing phase is initalized
    *
    * @return initialTime
    */
   double getInitialTime();

   /**
    * Intrinsic robot parameter.
    * <p>
    * Correspond the natural frequency response of the robot when modeled as an inverted pendulum:
    * {@code omega0 = Math.sqrt(g / z0)}, where {@code g} is equal to the magnitude of the gravity,
    * and {@code z0} is the constant center of mass height of the robot with respect to is feet.
    * </p>
    *
    * @param omega0 the robot's natural frequency.
    */
   void setOmega0(double omega0);

   /**
    * Returns whether the ICP planner currently assumes to that the robot is in double support.
    *
    * @return whether the ICP plan is in double support state or not.
    */
   boolean isInDoubleSupport();

   /**
    * Returns whether the ICP planner currently assumes to that the robot is standing.
    *
    * @return whether the ICP plan is in standing state or not.
    */
   boolean isInStanding();

   /**
    * Returns whether the ICP planner currently assumes to that the robot is performing the first
    * transfer of a walking sequence, i.e. just left standing state.
    *
    * @return whether the ICP plan is in initial transfer state or not.
    */
   boolean isInInitialTransfer();

   /**
    * Retrieves the desired ICP position at the end of the current state.
    *
    * @param finalDesiredCapturePointPositionToPack the final desired ICP position. Modified.
    */
   void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack);

   /**
    * Retrieves the desired ICP position at the end of the current state.
    *
    * @param finalDesiredCapturePointPositionToPack the final desired ICP position. Modified.
    */
   void getFinalDesiredCapturePointPosition(YoFramePoint2D finalDesiredCapturePointPositionToPack);

   /**
    * Retrieves the desired CoM position at the end of the current step.
    *
    * @param finalDesiredCenterOfMassPositionToPack the final desired ICP position. Modified.
    */
   void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack);

   /**
    * Retrieves the position of the next exit CMP.
    * <p>
    * This is typically useful to estimate where the robot will put its center of pressure at the
    * end of single support.
    * </p>
    *
    * @param exitCMPToPack the next exit CMP position. Modified.
    */
   void getNextExitCMP(FramePoint3D exitCMPToPack);

   /**
    * Tests if the ICP planner is done with the current state.
    *
    * @return {@code true} if the plan for the current state is done, returns {@code false}
    *         otherwise.
    */
   boolean isDone();

   /**
    * Tests the current state in the ICP plan results in having the desired CMP located at the exit
    * CMP.
    *
    * @return {@code true} if the current CMP is located on the exit CMP, returns {@code false}
    *         otherwise.
    */
   boolean isOnExitCMP();

   int getNumberOfFootstepsToConsider();

   int getNumberOfFootstepsRegistered();

   RobotSide getTransferToSide();

   double getOmega0();
}