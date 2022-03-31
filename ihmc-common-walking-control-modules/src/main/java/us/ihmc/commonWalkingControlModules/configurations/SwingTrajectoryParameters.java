package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class SwingTrajectoryParameters
{
   /**
    * Useful to force the swing foot to end up with an height offset with respect to the given
    * footstep.
    */
   public abstract double getDesiredTouchdownHeightOffset();

   /**
    * Useful to force the swing foot go towards the ground once the desired final position is reached
    * but the foot has not touched the ground yet.
    */
   public abstract double getDesiredTouchdownVelocity();

   /**
    * Useful to force the swing foot accelerate towards the ground once the desired final position is
    * reached but the foot has not touched the ground yet.
    */
   public abstract double getDesiredTouchdownAcceleration();

   /**
    * Ratio used to modify the x and y components of the desired swing final velocity by adding a
    * portion of the predicted center of mass velocity at the end of swing.
    * <p>
    * A value of 0 will not modify the touchdown velocity, while a value of 1 will add the full
    * predicted CoM velocity to the desired touchdown velocity .
    * </p>
    */
   public double getFinalCoMVelocityInjectionRatio()
   {
      return 0.0;
   }
   
   /**
    * Ratio used to modify the x and y components of the desired swing final acceleration by adding a
    * portion of the predicted center of mass acceleration at the end of swing.
    * <p>
    * A value of 0 will not modify the touchdown acceleration, while a value of 1 will add the full
    * predicted CoM acceleration to the desired touchdown acceleration .
    * </p>
    */
   public double getFinalCoMAccelerationInjectionRatio()
   {
      return 0.0;
   }

   /**
    * Useful to relax the desired touchdown velocity when computing the swing trajectory.
    * 
    * @return the weight to used with the swing final desired velocity.
    */
   public Tuple3DReadOnly getTouchdownVelocityWeight()
   {
      return new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   }

   /**
    * Z-offset used for footsteps that have height that is to be recomputed. The new height will be the
    * one of the support sole frame plus this offset.
    */
   public double getBlindFootstepsHeightOffset()
   {
      return 0.03;
   }

   /**
    * Returns a ratio to multiply the swing foot velocity adjustment when the swing trajectory is
    * modified online. 0.0 will eliminate any velocity adjustment. 1.0 will make it try to move to the
    * new trajectory in 1 dt.
    * 
    * @return damping ratio (0.0 to 1.0)
    */
   public double getSwingFootVelocityAdjustmentDamping()
   {
      return 0.0;
   }

   /**
    * Returns the percent of the step length which will be used to determine the swing waypoints.
    */
   public double[] getSwingWaypointProportions()
   {
      return new double[] {0.15, 0.85};
   }

   /**
    * Returns the percent of the step length which will be used to determine the swing waypoints when
    * taking a step of the type obstacle clearance
    */
   public double[] getObstacleClearanceProportions()
   {
      return new double[] {0.15, 0.85};
   }

   /**
    * Whether or not to add an orientation midpoint when doing trajectories of type obstacle clearance
    */
   public boolean addOrientationMidpointForObstacleClearance()
   {
      return false;
   }

   /**
    * Amount of interpolation between the initial orientation and the final orientation during obstacle
    * clearance. Is not used unless {@link #addOrientationMidpointForObstacleClearance()} returns true.
    */
   public double midpointOrientationInterpolationForObstacleClearance()
   {
      return 0.4;
   }

   /**
    * If a step up or a step down is executed, the swing trajectory will switch to the obstacle
    * clearance mode. The value defined here determines the threshold for the height difference between
    * current foot position and step position that causes this switch.
    */
   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return 0.04;
   }

   /**
    * Limits the swing foot motion according to the motion range.
    */
   public boolean useSingularityAvoidanceInSwing()
   {
      return true;
   }

   /**
    * Progressively limits the CoM height as the support leg(s) are getting straighter.
    */
   public boolean useSingularityAvoidanceInSupport()
   {
      return true;
   }
}
