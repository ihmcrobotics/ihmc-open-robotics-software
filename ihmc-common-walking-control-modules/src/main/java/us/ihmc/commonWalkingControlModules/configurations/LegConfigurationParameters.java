package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;

public class LegConfigurationParameters
{
   /**
    * This is the acceleration used to straighten the desire privileged configuration of the support leg's knee.
    * This is used whenever a leg is first loaded to straighten from the current configuration to the
    * straight configuration defined by {@link #getKneeAngleWhenStraight()}.
    *
    * @return knee rad/second^2 for straightening
    */
   public double getAccelerationForSupportKneeStraightening()
   {
      return 10.0;
   }

   public double getSupportKneeCollapsingDurationFractionOfStep()
   {
      return 0.5;
//      return 1.0;
//      return 0.1; // for big step down
   }

   /**
    * Angle to define what the knee privileged configuration considers straight.
    * This is used in the straight leg state by the support legs when the robot is attempting to walk with
    * straight legs, and also to help extend the leg at the end of the swing state.
    *
    * @return knee angle in radians
    */
   public double getKneeAngleWhenStraight()
   {
      return 0.25;
   }

   /**
    * Knee angle used when the leg is in its bracing state. The bracing state can occur when stepping down,
    * or recovering from large disturbances. This is used instead of {@link #getKneeAngleWhenExtended()} ()}.
    */
   public double getKneeAngleWhenBracing()
   {
      return 0.4;
   }

   /**
    * Knee angle used when the leg is fully extending during swing.
    */
   public double getKneeAngleWhenExtended()
   {
      return 0.0;
   }

   /**
    * Fraction of the knee midrange angle to consider the "collapsed" angle, to be used by the
    * {@link us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule.CollapseKneeControlState}
    */
   public double getDesiredFractionOfMidrangeForCollapsedAngle()
   {
      return 0.3;
//      return 1.8; // for big step down
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to being straight.
    * This is critical to allow the robot to "extend" the leg out to the next foothold.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToStraightenLeg()
   {
      return 0.4;
   }

   /**
    * Returns a fraction of the transfer state to switch the knee privileged configuration to bent.
    * This is important to start collapsing the leg to avoid being stuck in the singularity, allowing the
    * upcoming swing leg to start the swing motion naturally.
    *
    * @return fraction of transfer state (0.0 to 1.0)
    */
   public double getFractionOfTransferToCollapseLeg()
   {
      return 0.7;
//      return 1.0;
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to bent.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToCollapseStanceLeg()
   {
      return 0.55;
//      return 1.0;
//      return 0.3; //for big step down
   }

   /**
    * Determines whether or not to attempt to use straight legs when indirectly controlling the center of mass
    * height using the nullspace in the full task Jacobian.
    * This will not do anything noticeable unless {@link WalkingControllerParameters#enableHeightFeedbackControl()}
    * returns true, as that indicates whether or not to use the pelvis to control the center of mass height.
    *
    * @return boolean (true = try and straighten, false = do not try and straighten)
    */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   /**
    * These are the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the straight leg state or straightening state.
    * It contains the gains used by the {@link us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public LegConfigurationGains getStraightLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(40.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /**
    * This is the weight placed on the knee privileged joint accelerations or velocities when the leg is in
    * the bent leg state in the optimization. For a typical humanoid, these joints are the hip pitch and
    * ankle pitch.
    */
   public double getLegPrivilegedLowWeight()
   {
      return 5.0;
   }

   public double getLegPrivilegedMediumWeight()
   {
      return 50.0;
   }

   /**
    * This is the weight placed on the knee privileged joint accelerations or velocities when the leg is in
    * the straight leg state in the optimization.
    */
   public double getLegPrivilegedHighWeight()
   {
      return 150.0;
   }

   /**
    * These are the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the straight leg state or straightening state.
    * It contains the gains used by the {@link us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(40.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /**
    * The maximum privileged velocity to be allowed after feedback as an objective in the optimization, as calculated
    * by the {@link JointPrivilegedConfigurationHandler}.
    * This value is used for all the leg pitch joints, which, for a standard humanoid, is the hip pitch, knee pitch
    * and ankle pitch.
    *
    * @return max privileged velocity in radians per second.
    */
   public double getPrivilegedMaxVelocity()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * The maximum privileged acceleration to be allowed after feedback as an objective in the optimization, as
    * calculated by the {@link JointPrivilegedConfigurationHandler}.
    * This value is used for all the leg pitch joints, which, for a standard humanoid, is the hip pitch, knee pitch
    * and ankle pitch.
    *
    * @return max privileged acceleration in radians per second.
    */
   public double getPrivilegedMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }
}
