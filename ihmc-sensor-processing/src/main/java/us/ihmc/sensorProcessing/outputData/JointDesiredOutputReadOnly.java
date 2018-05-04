package us.ihmc.sensorProcessing.outputData;

/**
 * An interface for a data holder that is used to communicate desired joint behavior
 * (setpoints and controller properties) from a whole body controller to joint level
 * controllers.
 */
public abstract class JointDesiredOutputReadOnly
{
   public abstract boolean hasDesiredTorque();
   public abstract boolean hasDesiredPosition();
   public abstract boolean hasDesiredVelocity();
   public abstract boolean hasDesiredAcceleration();
   public abstract double getDesiredTorque();
   public abstract double getDesiredPosition();
   public abstract double getDesiredVelocity();
   public abstract double getDesiredAcceleration();
   public abstract boolean pollResetIntegratorsRequest();
   public abstract boolean peekResetIntegratorsRequest();

   /**
    * Returns true if a desired control mode was set for this joint.
    * @see #getControlMode()
    */
   public abstract boolean hasControlMode();

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired control scheme to be used by the joint controller. It can be used to
    * switch control laws, the tracked value (e.g. position, torque), or determine the actuator
    * control mode.
    */
   public abstract JointDesiredControlMode getControlMode();

   /**
    * Returns true if a desired stiffness was set for this joint.
    * @see #getStiffness()
    */
   public abstract boolean hasStiffness();

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired joint stiffness. This can be used by the joint controller to determine control gains.
    */
   public abstract double getStiffness();

   /**
    * Returns true if a desired damping was set for this joint.
    * @see #getDamping()
    */
   public abstract boolean hasDamping();

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired joint damping. This can be used by the joint controller to determine control gains.
    */
   public abstract double getDamping();

   /**
    * Returns true if a master gain was set for this joint.
    * @see #getMasterGain()
    */
   public abstract boolean hasMasterGain();

   /**
    * <p>
    * This is intended to specify the desired amount of feedback action in percent used to
    * control the joint. E.g. if the master gain is set to zero and the control mode is effort
    * the joint should be controlled using open loop effort.
    * </p>
    * The gain can be used for other purposes as originally intended to communicate information
    * from the whole body controller to the joint control level.
    */
   public abstract double getMasterGain();

   /**
    * Returns true if a velocity scaling was set for this joint.
    * @see #getVelocityScaling()
    */
   public abstract boolean hasVelocityScaling();

   /**
    * <p>
    * This allows to specify a desired velocity scaling for the joint level controller. In the
    * simplest for the joint control law contains a damping / velocity term that looks like this:</br>
    * damping * (velocityScaling * qd_d - qd)
    * </p>
    * By default this parameter should be set to 1.0 but can be set to a value between 0.0 and 1.0.
    * If set to zero the velocity tracking of this joint will be deactivated and the joint damping
    * will simulate viscous friction. If set to one the damping term will attempt to track the desired
    * velocity.
    */
   public abstract double getVelocityScaling();

   /**
    * Returns true if a break frequency for the integration of the desired acceleration to a desired
    * velocity was set for this joint.
    * @see #getVelocityIntegrationBreakFrequency()
    */
   public abstract boolean hasVelocityIntegrationBreakFrequency();

   /**
    * If the integration of desired accelerations is handled on the joint level this value allows
    * to specify a break frequency for the integration of the desired acceleration to a desired velocity.
    *
    * @return the value of this parameter needs to be between 0.0 and infinity.
    */
   public abstract double getVelocityIntegrationBreakFrequency();

   /**
    * Returns true if a break frequency for the integration of the desired acceleration to a desired
    * position was set for this joint.
    * @see #getPositionIntegrationBreakFrequency()
    */
   public abstract boolean hasPositionIntegrationBreakFrequency();

   /**
    * If the integration of desired accelerations is handled on the joint level this value allows
    * to specify a break frequency for the integration of the desired velocity to a desired position.
    *
    * @return the value of this parameter needs to be between 0.0 and infinity.
    */
   public abstract double getPositionIntegrationBreakFrequency();

   /**
    * Returns true if a maximum position error was set for this joint.
    *
    * @see #getMaxPositionError()
    */
   public abstract boolean hasMaxPositionError();

   /**
    * Gets the maximum position error to consider in the low level control of this joint. How this
    * value is used is specific to the joint low level control. In some cases it is used to limit
    * the acceleration integration in other cases it might be used to determine a maximum position
    * feedback for a joint PD controller.
    *
    * @return the maximum position error for the joint.
    */
   public abstract double getMaxPositionError();

   /**
    * Returns true if a maximum velocity error was set for this joint.
    *
    * @see #getMaxVelocityError()
    */
   public abstract boolean hasMaxVelocityError();

   /**
    * Gets the maximum velocity error to consider in the low level control of this joint. How this
    * value is used is specific to the joint low level control. In some cases it is used to limit
    * the acceleration integration in other cases it might be used to determine a maximum velocity
    * feedback for a joint PD controller.
    *
    * @return the maximum velocity error for the joint.
    */
   public abstract double getMaxVelocityError();

   @Override
   public String toString()
   {
      String ret = "Joint Desired Output:\n";
      if (hasControlMode())
         ret += "controlMode = " + getControlMode() + "\n";
      if (hasDesiredTorque())
         ret += "desiredTorque = " + getDesiredTorque() + "\n";
      if (hasDesiredPosition())
         ret += "desiredPosition = " + getDesiredPosition() + "\n";
      if (hasDesiredVelocity())
         ret += "desiredVelocity = " + getDesiredVelocity() + "\n";
      if (hasDesiredAcceleration())
         ret += "desiredAcceleration = " + getDesiredAcceleration() + "\n";
      if (hasMasterGain())
         ret += "masterGain = " + getMasterGain() + "\n";
      if (hasVelocityScaling())
         ret += "velocityScaling = " + getVelocityScaling() + "\n";
      if (hasVelocityIntegrationBreakFrequency())
         ret += "velocityIntegrationBreakFrequency = " + getVelocityIntegrationBreakFrequency() + "\n";
      if (hasPositionIntegrationBreakFrequency())
         ret += "positionIntegrationBreakFrequency = " + getPositionIntegrationBreakFrequency() + "\n";
      return ret;
   }
}