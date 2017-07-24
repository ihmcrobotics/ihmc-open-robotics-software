package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;

public class StraightLegWalkingParameters
{
   /**
    * This is the speed used to straighten the desire privileged configuration of the support leg's knee.
    * This is used whenever a leg is first loaded to straighten from the current configuration to the
    * straight configuration defined by {@link #getStraightKneeAngle()}.
    *
    * @return knee rad/second for straightening
    */
   public double getSpeedForSupportKneeStraightening()
   {
      return 0.2;
   }

   public double getSupportKneeCollapsingDuration()
   {
      return 0.5;
   }

   /**
    * Angle used to what it means to set the knee privileged configuration to straight.
    * This is used in the straight leg state by the support legs when the robot is attempting to walk with
    * straight legs, and also to help extend the leg at the end of the swing state.
    *
    * @return knee angle in radians
    */
   public double getStraightKneeAngle()
   {
      return 0.25;
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to being straight.
    * This is critical to allow the robot to "extend" the leg out to the next foothold.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToStraightenLeg()
   {
      return 0.5;
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
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to bent.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToCollapseStanceLeg()
   {
      return 0.7;
   }

   /**
    * Determines whether or not to attempt to use straight legs when indirectly controlling the center of mass
    * height using the nullspace in the full task Jacobian.
    * This will not do anything noticeable unless {@link WalkingControllerParameters#controlHeightWithMomentum()}
    * returns true, as that indicates whether or not to use the pelvis to control the center of mass height.
    *
    * @return boolean (true = try and straighten, false = do not try and straighten)
    */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   /**
    * This is the weight placed on the privileged joint accelerations or velocities for the other leg pitch
    * joints in the optimization. For a typical humanoid, these joints are the hip pitch and ankle pitch.
    * These additional degrees of freedom are important to stabilize the knee pitch joint when attempting
    * to stand and walk with straight legs.
    *
    * @return privileged configuration weight.
    */
   public double getLegPitchPrivilegedWeight()
   {
      return 5.0;
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
      gains.setActuatorSpaceKp(60.0);
      //gains.setActuatorSpaceKd(20.0);

      //gains.setUseActuatorSpacePositionControl(true);

      /*
      gains.setBlendPositionError(true);
      gains.setMaxPositionBlendingFactor(1.3);

      gains.setBlendVelocityError(true);
      gains.setMaxVelocityBlendingFactor(2.0);
      */

      return gains;
   }

   /**
    * This is the weight placed on the knee privileged joint accelerations or velocities when the leg is in
    * the straight leg state in the optimization. For a typical humanoid, these joints are the hip pitch and
    * ankle pitch.
    *
    * @return privileged configuration weight.
    */
   public double getKneeStraightLegPrivilegedWeight()
   {
      return 5.0;
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
      gains.setActuatorSpaceKp(60.0);
      gains.setActuatorSpaceKd(6.0);

      gains.setBlendPositionError(false);
      gains.setBlendVelocityError(false);

      return gains;
   }
   /**
    * This is the weight placed on the knee privileged joint accelerations or velocities when the leg is in
    * the bent leg state in the optimization. For a typical humanoid, these joints are the hip pitch and
    * ankle pitch.
    *
    * @return privileged configuration weight.
    */
   public double getKneeBentLegPrivilegedWeight()
   {
      return 5.0;
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
