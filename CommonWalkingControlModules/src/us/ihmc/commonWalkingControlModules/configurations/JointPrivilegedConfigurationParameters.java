package us.ihmc.commonWalkingControlModules.configurations;

public class JointPrivilegedConfigurationParameters
{
   public JointPrivilegedConfigurationParameters()
   {
   }

   /**
    * <p>
    * Returns the desired default configuration gain for the joint privileged configuration handler.
    * </p>
    * <p>
    * This gain is used to track the privileged configuration when no other gain is provided.
    * </p>
    * @return configuration gain
    */
   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   /**
    * <p>
    * Returns the desired default velocity gain for the joint privileged configuration handler.
    * </p>
    * <p>
    * This gain is used to damp the velocities when no other velocity gain is provided.
    * </p>
    * <p>
    * Note that when computing privileged velocities, this value is not used.
    * </p>
    * @return velocity gain
    */
   public double getDefaultVelocityGain()
   {
      return 6.0;
   }

   /**
    * <p>
    * Returns the default maximum velocity for the joint privileged configuration handler.
    * </p>
    * <p>
    * This limits the maximum velocity when computing privileged velocity commands that will be allowed to attempt
    * to reach the privileged configuration.
    * </p>
    * @return max joint velocity (rad / s)
    */
   public double getDefaultMaxVelocity()
   {
      return 2.0;
   }

   /**
    * <p>
    * Returns the default maximum acceleration for the joint privileged configuration handler.
    * </p>
    * <p>
    * This limits the maximum acceleration when computing privileged acceleration commands that will be allowed to attempt
    * to reach the privileged configuration.
    * </p>
    * @return max joint acceleration (rad / s^2)
    */
   public double getDefaultMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * <p>
    * Returns the default weight for the joint privileged configuration handler.
    * </p>
    * <p>
    * This is the weight placed on the privileged configuration command in the nullspace of the task Jacobian
    * </p>
    * @return privileged command weight
    */
   public double getDefaultWeight()
   {
      return 5.0;
   }

   /**
    * <p>
    *    Returns the configuration gain corresponding to {@link #getDefaultConfigurationGain()} to be utilized for the knee
    *    when the foot is in the support contact state.
    * </p>
    * @return configuration gain
    */
   public double getSupportKneeConfigurationGain()
   {
      return getDefaultConfigurationGain();
   }

   /**
    * <p>
    *    Returns the velocity gain corresponding to {@link #getDefaultVelocityGain()} to be utilized for the knee
    *    when the foot is in the support contact state.
    * </p>
    * @return velocity gain
    */
   public double getSupportKneeVelocityGain()
   {
      return getDefaultVelocityGain();
   }

   /**
    * <p>
    *    Returns the privileged configuration weight corresponding to {@link #getDefaultWeight()} to be utilized for the knee
    *    when the foot is in the support contact state.
    * </p>
    * @return privileged command weight
    */
   public double getSupportKneeWeight()
   {
      return getDefaultWeight();
   }

   /**
    * <p>
    *    Returns the configuration gain corresponding to {@link #getDefaultConfigurationGain()} to be utilized for the knee
    *    when the foot is in the unconstrained contact state.
    * </p>
    * @return configuration gain
    */
   public double getUnconstrainedKneeConfigurationGain()
   {
      return getDefaultConfigurationGain();
   }

   /**
    * <p>
    *    Returns the velocity gain corresponding to {@link #getDefaultVelocityGain()} to be utilized for the knee
    *    when the foot is in the unconstrained contact state.
    * </p>
    * @return velocity gain
    */
   public double getUnconstrainedKneeVelocityGain()
   {
      return getDefaultVelocityGain();
   }

   /**
    * <p>
    *    Returns the privileged configuration weight corresponding to {@link #getDefaultWeight()} to be utilized for the knee
    *    when the foot is in the unconstrained contact state.
    * </p>
    * @return privileged configuration weight
    */
   public double getUnconstrainedKneeWeight()
   {
      return getDefaultWeight();
   }

   /**
    * <p>
    *    Returns the configuration gain corresponding to {@link #getDefaultConfigurationGain()} to be utilized for the knee
    *    when the foot is in the on toes contact state.
    * </p>
    * @return configuration gain
    */
   public double getOnToesKneeConfigurationGain()
   {
      return getDefaultConfigurationGain();
   }

   /**
    * <p>
    *    Returns the velocity gain corresponding to {@link #getDefaultVelocityGain()} to be utilized for the knee
    *    when the foot is in the on toes contact state.
    * </p>
    * @return velocity gain
    */
   public double getOnToesKneeVelocityGain()
   {
      return getDefaultVelocityGain();
   }

   /**
    * <p>
    *    Returns the privileged configuration weight corresponding to {@link #getDefaultWeight()} to be utilized for the knee
    *    when the foot is in the on toes contact state.
    * </p>
    * @return privileged configuration weight
    */
   public double getOnToesKneeWeight()
   {
      return getDefaultWeight();
   }
}
