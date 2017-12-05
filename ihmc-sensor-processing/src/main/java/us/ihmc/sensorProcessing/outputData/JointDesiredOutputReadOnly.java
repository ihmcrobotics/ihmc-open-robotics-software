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
    * Returns true if a leak rate for the integration of the desired acceleration to a desired
    * velocity was set for this joint.
    * @see #getVelocityIntegrationLeakRate()
    */
   public abstract boolean hasVelocityIntegrationLeakRate();

   /**
    * <p>
    * If the integration of desired accelerations is handled on the joint level this value allows
    * to specify a leak rate for the integration of the desired acceleration to a desired velocity.
    * In its simplest form the integration equation might look like this:</br>
    * qd_desired(T+dt) = velocityLeak * qd(T) + dt * qdd_desired
    * </p>
    * The value of this parameter needs to be between 0.0 and 1.0.
    */
   public abstract double getVelocityIntegrationLeakRate();

   /**
    * Returns true if a leak rate for the integration of the desired acceleration to a desired
    * position was set for this joint.
    * @see #getPositionIntegrationLeakRate()
    */
   public abstract boolean hasPositionIntegrationLeakRate();

   /**
    * <p>
    * If the integration of desired accelerations is handled on the joint level this value allows
    * to specify a leak rate for the integration of the desired acceleration and the desired
    * velocity to a desired position. In its simplest form the integration equation might look like
    * this:</br>
    * q_desired(T+dt) = leakPosition * (q_desired(T) + dt * qd_desired(T)) + (1.0 - leakPosition) * q(T)</br>
    * Here, the desired position is leaking towards the actual (measured) joint position.
    * </p>
    * The value of this parameter needs to be between 0.0 and 1.0.
    */
   public abstract double getPositionIntegrationLeakRate();

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
      if (hasVelocityIntegrationLeakRate())
         ret += "velocityIntegrationLeakRate = " + getVelocityIntegrationLeakRate() + "\n";
      if (hasPositionIntegrationLeakRate())
         ret += "positionIntegrationLeakRate = " + getPositionIntegrationLeakRate() + "\n";
      return ret;
   }
}