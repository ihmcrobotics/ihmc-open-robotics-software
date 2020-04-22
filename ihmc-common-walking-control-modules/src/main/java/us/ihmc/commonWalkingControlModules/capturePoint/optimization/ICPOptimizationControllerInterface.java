package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.util.List;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPOptimizationControllerInterface
{
   void clearPlan();

   void setTransferDuration(double duration);
   void setSwingDuration(double duration);
   void setNextTransferDuration(double duration);

   void setFinalTransferDuration(double finalTransferDuration);

   void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon);

   void addFootstepToPlan(SimpleAdjustableFootstep footstep, double swingDuration, double transferDuration);

   /**
    * Lets the controller know that it just entered the standing state. This does things
    * like resets the contact constraints and the clocks.
    *
    * @param initialTime the current time, or the time that the robot started the
    *                    standing state.
    */
   void initializeForStanding(double initialTime);

   /**
    * Lets the controller know that it just entered the transfer state. This does things
    * like resets the contact constraints and the clocks.
    *
    * @param initialTime the current time, or the time that the robot started the
    *                    transfer state.
    * @param transferToSide the side that the robot is transferring to. This is the
    *                       upcoming support side.
    */
   void initializeForTransfer(double initialTime, RobotSide transferToSide);

   /**
    * Lets the controller know that it just entered the swing state. This does things
    * like resets the contact constraints and the clocks.
    *
    * @param initialTime the current time, or the time that the robot started the
    *                    transfer state.
    * @param supportSide the side that the robot is transferring to. This is the
    *                    upcoming support side.
    * @param omega0 the current natural frequency of the inverted pendulum.
    */
   void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0);

   /**
    * Packs the desired CMP location computed by the ICP controller. The CMP to pack is
    * expected to be in world frame.
    *
    * @param desiredCMPToPack where the CMP location is stored. Modified.
    * @throws ReferenceFrameMismatchException if the desiredCMPToPack is not in world frame.
    */
   void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack);

   /**
    * Packs the desired CoP location computed by the ICP controller. The CoP to pack is
    * expected to be in world frame.
    *
    * @param desiredCoPToPack where the CoP location is stored. Modified.
    * @throws ReferenceFrameMismatchException if the desiredCMPToPack is not in world frame.
    */
   void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack);

   /**
    * Gets the desired Footstep pose computed by the ICP controller.
    * @return pose of the footstep solution.
    */
   FramePose3DReadOnly getFootstepSolution();

   /**
    * Returns whether or not the controller adjusted the footstep this tick.
    */
   boolean wasFootstepAdjusted();

   /**
    * Returns whether or not the controller is using angular momentum.
    */
   boolean useAngularMomentum();

   /**
    * Returns whether or not the controller is using step adjustment.
    */
   boolean useStepAdjustment();

   /**
    * Computes the current feedback problem. This method just does CMP feedback, assuming that there is not
    * a desired value of angular momentum. If there is a desired angular momentum, the CoP must also be
    * specified, and you should use {@link #compute(double, FramePoint2DReadOnly, FrameVector2DReadOnly,
    * FramePoint2DReadOnly, FrameVector2DReadOnly, FramePoint2DReadOnly, FrameVector2DReadOnly, double)}
    *
    * @param currentTime the current robot time. Not Modified.
    * @param desiredICP the current desired ICP position. Not Modified.
    * @param desiredICPVelocity the current desired ICP velocity. Not Modified.
    * @param desiredCoP the current desired CoP position. Corresponds with the current
    *                   desired CMP position. Not Modified.
    * @param currentICP the current estimated ICP position. Not Modified.
    * @param currentICPVelocity the current estimated ICP velocity. This is used to estimate
    *                           if the robot is "stuck". Can be noisy, so be careful. Not Modified.
    * @param omega0 current natural frequency of the inverted pendulum.
    */
   void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity,
                FramePoint2DReadOnly desiredCoP, FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0);

   /**
    * Computes the current feedback problem. This method does CoP and CMP feedback, with the desired angular
    * momentum encoded into the {@param desiredCMPOffset}.
    *
    * @param currentTime the current robot time. Not Modified.
    * @param desiredICP the current desired ICP position. Not Modified.
    * @param desiredICPVelocity the current desired ICP velocity. Not Modified.
    * @param desiredCoP the current desired CoP position. Corresponds with the current
    *                   desired CMP position. Not Modified.
    * @param desiredCMPOffset distance from the desired CoP to the desired CMP. Not Modified.
    * @param currentICP the current estimated ICP position. Not Modified.
    * @param currentICPVelocity the current estimated ICP velocity. This is used to estimate
    *                           if the robot is "stuck". Can be noisy, so be careful. Not Modified.
    * @param omega0 current natural frequency of the inverted pendulum.
    */
   void compute(double currentTime, FramePoint2DReadOnly desiredICP, FrameVector2DReadOnly desiredICPVelocity,
                FramePoint2DReadOnly desiredCoP, FrameVector2DReadOnly desiredCMPOffset,
                FramePoint2DReadOnly currentICP, FrameVector2DReadOnly currentICPVelocity, double omega0);

   /**
    * Submit the remaining time in the swing state. This duration can be modified
    * to "speed up" the ICP plan if the error is in the direction of the dynamics,
    * or to "slow down" the ICP plan if the ICP error is lagging.
    */
   void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   /**
    * Submits the current planar regions that describe the environment. This could
    * be useful if constraining the step adjustment, or taking into account the
    * environment for the step adjustment.
    *
    * @param planarRegions list of the current planar regions that describe the environment.
    */
   void submitCurrentPlanarRegions(List<PlanarRegion> planarRegions);

   /**
    * This controller combines both ICP error feedback and step adjustment. Step adjustment can be though
    * of as reducing the ICP error, as less feedback is needed when there is step adjustment. This method
    * returns that error reduction, which can be thought of as "shifting" the desired ICP.
    *
    * @return vector of the amount of adjustment to the desired ICP (m).
    */
   FrameVector3DReadOnly getICPShiftFromStepAdjustment();
}
