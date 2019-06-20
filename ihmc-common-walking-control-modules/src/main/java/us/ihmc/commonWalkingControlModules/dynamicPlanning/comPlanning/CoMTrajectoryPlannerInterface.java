package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.List;

public interface CoMTrajectoryPlannerInterface
{
   /**
    * Sets the nominal CoM height to be used by the planner.
    */
   void setNominalCoMHeight(double nominalCoMHeight);

   /**
    * Solves for the desired center of mass trajectory.
    */
   void solveForTrajectory(List<? extends ContactStateProvider> contactSequence);

   /**
    * Computes the desired values for the current segment at time {@param timeInPhase}.
    * @param timeInPhase time in the current phase. Note that this assumes that the phase starts at 0.0.
    */
   default void compute(double timeInPhase)
   {
      compute(0, timeInPhase);
   }

   /**
    * Computes the desired values for the segment {@param segmentId} at time {@param timeInPhase}.
    * @param timeInPhase time in the current phase. Note that this assumes that the phase starts at 0.0.
    */
   void compute(int segmentId, double timeInPhase);

   void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                FixedFramePoint3DBasics vrpPositionToPack);

   /**
    * Sets the initial center of mass position. Note that this should be set at every change in contact! This sets the initial
    * boundary condition for the trajectory planning.
    */
   void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity);

   /**
    * Gets the desired position of the Divergent Component of Motion computed in {@link #compute(double)}.
    */
   FramePoint3DReadOnly getDesiredDCMPosition();

   /**
    * Gets the desired velocity of the Divergent Component of Motion computed in {@link #compute(double)}.
    */
   FrameVector3DReadOnly getDesiredDCMVelocity();

   /**
    * Gets the desired position of the Center of Mass computed in {@link #compute(double)}.
    */
   FramePoint3DReadOnly getDesiredCoMPosition();

   /**
    * Gets the desired velocity of the Center of Mass computed in {@link #compute(double)}.
    */
   FrameVector3DReadOnly getDesiredCoMVelocity();

   /**
    * Gets the desired acceleration of the Center of Mass computed in {@link #compute(double)}.
    */
   FrameVector3DReadOnly getDesiredCoMAcceleration();

   /**
    * Gets the desired position of the Virtual Repellent Point computed in {@link #compute(double)}.
    */
   FramePoint3DReadOnly getDesiredVRPPosition();

}
