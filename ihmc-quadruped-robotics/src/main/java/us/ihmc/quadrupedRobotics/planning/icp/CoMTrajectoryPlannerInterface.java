package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CoMTrajectoryPlannerInterface
{
   /**
    * Sets the nominal CoM height to be used by the planner.
    */
   void setNominalCoMHeight(double nominalCoMHeight);

   /**
    * Solves for the desired center of mass trajectory.
    */
   void solveForTrajectory();

   /**
    * Computes the desired values for the current segment at time {@param timeInPhase}.
    * @param timeInPhase time in the current phase. Note that this assumes that the phase starts at 0.0.
    */
   void compute(double timeInPhase);

   /**
    * Sets the current center of mass position. Note that this should be set at every change in contact! This sets the initial
    * boundary condition for the trajectory planning.
    */
   void setCurrentCoMPosition(FramePoint3DReadOnly currentComPosition);

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

   /**
    * Gets the desired position of the Enhanced Centroidal Momentum Pivot point computed in {@link #compute(double)}.
    */
   FramePoint3DReadOnly getDesiredECMPPosition();
}
