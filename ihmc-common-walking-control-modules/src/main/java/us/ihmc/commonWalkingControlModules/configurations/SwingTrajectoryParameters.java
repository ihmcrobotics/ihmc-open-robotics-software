package us.ihmc.commonWalkingControlModules.configurations;

public abstract class SwingTrajectoryParameters
{
   public abstract boolean doToeTouchdownIfPossible();

   public abstract double getToeTouchdownAngle();

   /**
    * When stepping down, and we want to do toe strike, this ratio is used to determine the toe touchdown angle. This ratio is used to multiply the stepping
    * depth to determine the toe touchdown angle. This touchdown angle is then clipped to above and below the value returned by {@link #getToeTouchdownAngle()}.
    * @return touchdown depth ratio
    */
   public double getToeTouchdownDepthRatio()
   {
      return 5.0;
   }

   /**
    * Returns the minimum distance stepping down that will be used to do toe touchdown if {@link #doToeTouchdownIfPossible()} is enabled.
    * @return minimum step down height (m).
    */
   public double getStepDownHeightForToeTouchdown()
   {
      return -0.05;
   }

   public abstract boolean doHeelTouchdownIfPossible();

   public abstract double getHeelTouchdownAngle();

   /**
    * When stepping over terrain of the correct height, and we want to do heel strike, this ratio is used to determine the heel touchdown angle.
    * This ratio is used to multiply the step length to determine the heel touchdown angle. This touchdown angle is then clipped to above and
    * below the value returned by {@link #getHeelTouchdownAngle()}.
    * @return touchdown length ratio.
    */
   public double getHeelTouchdownLengthRatio()
   {
      return 0.5;
   }

   /**
    * Returns the maximum height that heel touchdown will be used if {@link #doHeelTouchdownIfPossible()} is enabled.
    * @return maximum height (m).
    */
   public double getMaximumHeightForHeelTouchdown()
   {
      return 0.10;
   }

   /** Useful to force the swing foot to end up with an height offset with respect to the given footstep. */
   public abstract double getDesiredTouchdownHeightOffset();

   /** Useful to force the swing foot go towards the ground once the desired final position is reached but the foot has not touched the ground yet. */
   public abstract double getDesiredTouchdownVelocity();

   /** Useful to force the swing foot accelerate towards the ground once the desired final position is reached but the foot has not touched the ground yet. */
   public abstract double getDesiredTouchdownAcceleration();

   /** Z-offset used for footsteps that have height that is to be recomputed. The new height will be the one of the support sole frame plus this offset. */
   public double getBlindFootstepsHeightOffset()
   {
      return 0.03;
   }

   /**
    * Returns a ratio to multiply the swing foot velocity adjustment when the swing trajectory is modified online.
    * 0.0 will eliminate any velocity adjustment.
    * 1.0 will make it try to move to the new trajectory in 1 dt.
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
    * Returns the percent of the step length which will be used to determine the swing waypoints when taking a step
    * of the type obstacle clearance
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
    * Amount of interpolation between the initial orientation and the final orientation during obstacle clearance.
    * Is not used unless {@link #addOrientationMidpointForObstacleClearance()} returns true.
    */
   public double midpointOrientationInterpolationForObstacleClearance()
   {
      return 0.4;
   }

   /**
    * If a step up or a step down is executed, the swing trajectory will switch to the obstacle clearance
    * mode. The value defined here determines the threshold for the height difference between current foot
    * position and step position that causes this switch.
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

   public abstract double getMinMechanicalLegLength();
}
