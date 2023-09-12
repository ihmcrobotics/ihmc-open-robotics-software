package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class SwingTrajectoryParameters
{
   public abstract double getMaxSwingHeightFromStanceFoot();

   /**
    * Returns the minimum swing height from the stance foot for this robot
    */
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.1;
   }

   /**
    * Default swing height used by the controller. If a swing height is not specified or lies outside
    * of the allowable range, this value is used.
    */
   public double getDefaultSwingHeightFromStanceFoot()
   {
      return getMinSwingHeightFromStanceFoot();
   }

   /**
    * Custom waypoint positions are precomputed externally. During execution the initial foot pose
    * might be different than expected, and the preplanned trajectory might have the foot go backward
    * before moving forward, for example. This provides a threshold for the maximum angle from forward
    * to use - lower is more restrictive, 90 deg max recommended.
    */
   public double getCustomWaypointAngleThreshold()
   {
      return Math.toRadians(50.0);
   }

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

   public double getMinLiftOffVerticalVelocity()
   {
      return 0.0;
   }

   /**
    * When this value is 0.0, the initial foot velocity is 0.0. When it is 1.0, the initial swing foot
    * velocity is the pelvis velocity.
    */
   public double getPelvisVelocityInjectionRatio()
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
