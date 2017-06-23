package us.ihmc.commonWalkingControlModules.configurations;

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
      return 0.25;
   }

   public double getSupportKneeCollapsingDuration()
   {
      return 0.5;
   }

   /**
    * If set to true, the privileged acceleration for the leg pitch joints will be determined by blending feedback
    * based on both the joint space error and a virtual actuator length error. If the leg is desired to be fully extended,
    * it will use entirely virtual actuator feedback. If the knee setpoint is at the midpoint, it will use entirely joint
    * space feedback.
    * @return whether or not to blend jointspace with virtual actuator feedback
    */
   public boolean blendPrivilegedConfigurationPositionError()
   {
      return false;
   }

   /**
    * If set to true, the privileged acceleration for the leg pitch joints will be determined by blending feedback
    * based on both the joint space velocity error and a virtual actuator velocity error. If the leg is desired to be fully extended,
    * it will use entirely virtual actuator feedback. If the knee setpoint is at the midpoint, it will use entirely joint
    * space feedback.
    * @return whether or not to blend jointspace with virtual actuator feedback
    */
   public boolean blendPrivilegedConfigurationVelocityError()
   {
      return false;
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
      return 0.2;
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to being straight.
    * This is critical to allow the robot to "extend" the leg out to the next foothold.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToStraightenLeg()
   {
      return 0.8;
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
      return 0.9;
   }

   /**
    * Returns a fraction of the swing state to switch the knee privileged configuration to bent.
    *
    * @return fraction of swing state (0.0 to 1.0)
    */
   public double getFractionOfSwingToCollapseStanceLeg()
   {
      return Double.POSITIVE_INFINITY;
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
    * This is the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the straight leg state or straightening state.
    * It is the proportional gain used by the {@link JointPrivilegedConfigurationHandler} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public double getStraightLegJointSpacePrivilegedConfigurationGain()
   {
      return 40.0;
   }

   /**
    * This is the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the straight leg state or straightening state.
    * It is the proportional gain used by the {@link JointPrivilegedConfigurationHandler} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public double getStraightLegActuatorSpacePrivilegedConfigurationGain()
   {
      return 60.0;
   }

   /**
    * This is the velocity gain used to damp the knee privileged joint accelerations when the leg is in the straight
    * leg state.
    * This is the velocity gain used by the {@link JointPrivilegedConfigurationHandler} to damp the privileged
    * accelerations to project into the nullspace of the full task Jacobian. Note that if using the inverse kinematics
    * module, this gain does nothing, as that is determining privileged joint velocities rather than privileged
    * joint accelerations.
    *
    * @return privileged velocity gain.
    */
   public double getStraightLegJointSpacePrivilegedVelocityGain()
   {
      return 6.0;
   }

   /**
    * This is the velocity gain used to damp the knee privileged joint accelerations when the leg is in the straight
    * leg state.
    * This is the velocity gain used by the {@link JointPrivilegedConfigurationHandler} to damp the privileged
    * accelerations to project into the nullspace of the full task Jacobian. Note that if using the inverse kinematics
    * module, this gain does nothing, as that is determining privileged joint velocities rather than privileged
    * joint accelerations.
    *
    * @return privileged velocity gain.
    */
   public double getStraightLegActuatorSpacePrivilegedVelocityGain()
   {
      return 6.0;
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
    * This is the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the bent leg state.
    * It is the proportional gain used by the {@link JointPrivilegedConfigurationHandler} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public double getBentLegJointSpacePrivilegedConfigurationGain()
   {
      return 40.0;
   }

   /**
    * This is the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the bent leg state.
    * It is the proportional gain used by the {@link JointPrivilegedConfigurationHandler} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public double getBentLegActuatorSpacePrivilegedConfigurationGain()
   {
      return 60.0;
   }

   /**
    * This is the velocity gain used to damp the knee privileged joint accelerations when the leg is in the bent
    * leg state.
    * This is the velocity gain used by the {@link JointPrivilegedConfigurationHandler} to damp the privileged
    * accelerations to project into the nullspace of the full task Jacobian. Note that if using the inverse kinematics
    * module, this gain does nothing, as that is determining privileged joint velocities rather than privileged
    * joint accelerations.
    *
    * @return privileged velocity gain.
    */
   public double getBentLegJointSpacePrivilegedVelocityGain()
   {
      return 6.0;
   }

   /**
    * This is the velocity gain used to damp the knee privileged joint accelerations when the leg is in the bent
    * leg state.
    * This is the velocity gain used by the {@link JointPrivilegedConfigurationHandler} to damp the privileged
    * accelerations to project into the nullspace of the full task Jacobian. Note that if using the inverse kinematics
    * module, this gain does nothing, as that is determining privileged joint velocities rather than privileged
    * joint accelerations.
    *
    * @return privileged velocity gain.
    */
   public double getBentLegActuatorSpacePrivilegedVelocityGain()
   {
      return 6.0;
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
