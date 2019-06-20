package us.ihmc.sensorProcessing.outputData;

/**
 * An interface for a data holder that is used to communicate desired joint behavior (setpoints and
 * controller properties) from a whole body controller to joint level controllers.
 */
public interface JointDesiredOutputReadOnly
{

   default boolean hasDesiredTorque()
   {
      return !Double.isNaN(getDesiredTorque());
   }

   default boolean hasDesiredPosition()
   {
      return !Double.isNaN(getDesiredPosition());
   }

   default boolean hasDesiredVelocity()
   {
      return !Double.isNaN(getDesiredVelocity());
   }

   default boolean hasDesiredAcceleration()
   {
      return !Double.isNaN(getDesiredAcceleration());
   }

   double getDesiredTorque();

   double getDesiredPosition();

   double getDesiredVelocity();

   double getDesiredAcceleration();

   boolean pollResetIntegratorsRequest();

   boolean peekResetIntegratorsRequest();

   /**
    * Returns true if a desired control mode was set for this joint.
    * 
    * @see #getControlMode()
    */

   default boolean hasControlMode()
   {
      return getControlMode() != null;
   }

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired control scheme to be used by the joint controller. It can be used to switch
    * control laws, the tracked value (e.g. position, torque), or determine the actuator control mode.
    */
   JointDesiredControlMode getControlMode();

   /**
    * Returns true if a desired stiffness was set for this joint.
    * 
    * @see #getStiffness()
    */
   default boolean hasStiffness()
   {
      return !Double.isNaN(getStiffness());
   }

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired joint stiffness. This can be used by the joint controller to determine
    * control gains.
    */
   double getStiffness();

   /**
    * Returns true if a desired damping was set for this joint.
    * 
    * @see #getDamping()
    */
   default boolean hasDamping()
   {
      return !Double.isNaN(getDamping());
   }

   /**
    * <p>
    * The whole body controller can use this to provide information about desired joint behavior.
    * </p>
    * Specifies the desired joint damping. This can be used by the joint controller to determine
    * control gains.
    */
   double getDamping();

   /**
    * Returns true if a master gain was set for this joint.
    * 
    * @see #getMasterGain()
    */
   default boolean hasMasterGain()
   {
      return !Double.isNaN(getMasterGain());
   }

   /**
    * <p>
    * This is intended to specify the desired amount of feedback action in percent used to control the
    * joint. E.g. if the master gain is set to zero and the control mode is effort the joint should be
    * controlled using open loop effort.
    * </p>
    * The gain can be used for other purposes as originally intended to communicate information from
    * the whole body controller to the joint control level.
    */
   double getMasterGain();

   /**
    * Returns true if a velocity scaling was set for this joint.
    * 
    * @see #getVelocityScaling()
    */
   default boolean hasVelocityScaling()
   {
      return !Double.isNaN(getVelocityScaling());
   }

   /**
    * <p>
    * This allows to specify a desired velocity scaling for the joint level controller. In the simplest
    * for the joint control law contains a damping / velocity term that looks like this:</br>
    * damping * (velocityScaling * qd_d - qd)
    * </p>
    * By default this parameter should be set to 1.0 but can be set to a value between 0.0 and 1.0. If
    * set to zero the velocity tracking of this joint will be deactivated and the joint damping will
    * simulate viscous friction. If set to one the damping term will attempt to track the desired
    * velocity.
    */
   double getVelocityScaling();

   /**
    * Returns true if a break frequency for the integration of the desired acceleration to a desired
    * velocity was set for this joint.
    * 
    * @see #getVelocityIntegrationBreakFrequency()
    */
   default boolean hasVelocityIntegrationBreakFrequency()
   {
      return !Double.isNaN(getVelocityIntegrationBreakFrequency());
   }

   /**
    * If the integration of desired accelerations is handled on the joint level this value allows to
    * specify a break frequency for the integration of the desired acceleration to a desired velocity.
    *
    * @return the value of this parameter needs to be between 0.0 and infinity.
    */
   double getVelocityIntegrationBreakFrequency();

   /**
    * Returns true if a break frequency for the integration of the desired acceleration to a desired
    * position was set for this joint.
    * 
    * @see #getPositionIntegrationBreakFrequency()
    */
   default boolean hasPositionIntegrationBreakFrequency()
   {
      return !Double.isNaN(getPositionIntegrationBreakFrequency());
   }

   /**
    * If the integration of desired accelerations is handled on the joint level this value allows to
    * specify a break frequency for the integration of the desired velocity to a desired position.
    *
    * @return the value of this parameter needs to be between 0.0 and infinity.
    */
   double getPositionIntegrationBreakFrequency();

   /**
    * Returns true if a maximum position error was set for this joint.
    *
    * @see #getMaxPositionError()
    */
   default boolean hasMaxPositionError()
   {
      return !Double.isNaN(getMaxPositionError());
   }

   /**
    * Gets the maximum position error to consider in the low level control of this joint. How this
    * value is used is specific to the joint low level control. In some cases it is used to limit the
    * acceleration integration in other cases it might be used to determine a maximum position feedback
    * for a joint PD controller.
    *
    * @return the maximum position error for the joint.
    */
   double getMaxPositionError();

   /**
    * Returns true if a maximum velocity error was set for this joint.
    *
    * @see #getMaxVelocityError()
    */
   default boolean hasMaxVelocityError()
   {
      return !Double.isNaN(getMaxVelocityError());
   }

   /**
    * Gets the maximum velocity error to consider in the low level control of this joint. How this
    * value is used is specific to the joint low level control. In some cases it is used to limit the
    * acceleration integration in other cases it might be used to determine a maximum velocity feedback
    * for a joint PD controller.
    *
    * @return the maximum velocity error for the joint.
    */
   double getMaxVelocityError();

   default String getRepresentativeString()
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

   default boolean equals(JointDesiredOutputReadOnly other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (getControlMode() != other.getControlMode())
            return false;
         if (Double.compare(getDesiredTorque(), other.getDesiredTorque()) != 0)
            return false;
         if (Double.compare(getDesiredPosition(), other.getDesiredPosition()) != 0)
            return false;
         if (Double.compare(getDesiredVelocity(), other.getDesiredVelocity()) != 0)
            return false;
         if (Double.compare(getDesiredAcceleration(), other.getDesiredAcceleration()) != 0)
            return false;
         if (peekResetIntegratorsRequest() != other.peekResetIntegratorsRequest())
            return false;
         if (Double.compare(getStiffness(), other.getStiffness()) != 0)
            return false;
         if (Double.compare(getDamping(), other.getDamping()) != 0)
            return false;
         if (Double.compare(getMasterGain(), other.getMasterGain()) != 0)
            return false;
         if (Double.compare(getVelocityScaling(), other.getVelocityScaling()) != 0)
            return false;
         if (Double.compare(getVelocityIntegrationBreakFrequency(), other.getVelocityIntegrationBreakFrequency()) != 0)
            return false;
         if (Double.compare(getPositionIntegrationBreakFrequency(), other.getPositionIntegrationBreakFrequency()) != 0)
            return false;
         if (Double.compare(getMaxVelocityError(), other.getMaxVelocityError()) != 0)
            return false;
         if (Double.compare(getMaxPositionError(), other.getMaxPositionError()) != 0)
            return false;
         return true;
      }
   }
}