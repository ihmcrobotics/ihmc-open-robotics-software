package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface CartesianTrajectoryGenerator extends Finishable
{
   /**
    * initializes the trajectory generator with an initial position and velocity, final position and ground height.
    * @param initialPosition initial position of the trajectory
    * @param initialVelocity initial velocity of the trajectory
    * @param initialAcceleration initial acceleration of the trajectory
    * @param finalDesiredPosition final desired position of the trajectory (can be updated using updateFinalDesiredPosition later)
    * @param finalDesiredVelocity final desired velocity of the trajectory
    */
   public abstract void initialize(FramePoint3D initialPosition, FrameVector3D initialVelocity, FrameVector3D initialAcceleration, FramePoint3D finalDesiredPosition,
                                   FrameVector3D finalDesiredVelocity);

   /**
    * Packs the new desired position, velocity and acceleration.
    * @param positionToPack new desired position to pack
    * @param velocityToPack new desired velocity to pack
    * @param accelerationToPack new desired acceleration to pack
    * @param deltaT time step
    */
   public abstract void computeNextTick(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack, double deltaT);

   /**
    * Changes the final desired position for the trajectory
    * @param finalDesiredPosition the new final desired position
    */
   public abstract void updateFinalDesiredPosition(FramePoint3D finalDesiredPosition);

   public abstract ReferenceFrame getReferenceFrame();

   public abstract boolean isDone();

   public abstract double getFinalTime();
}
