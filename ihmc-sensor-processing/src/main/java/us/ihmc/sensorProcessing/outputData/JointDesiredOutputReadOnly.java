package us.ihmc.sensorProcessing.outputData;

import controller_msgs.msg.dds.JointDesiredOutputMessage;
import us.ihmc.commons.MathTools;

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

   default boolean hasLoadMode()
   {
      return getLoadMode() != null;
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
    * Specifies whether the joint associated to this output is used to support the robot weight. It can
    * be used to do some gain scheduling.
    */
   JointDesiredLoadMode getLoadMode();

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
    * Returns true if a maximum position error for acceleration integration was set for this joint.
    *
    * @see #getPositionIntegrationMaxError()
    */
   default boolean hasPositionIntegrationMaxError()
   {
      return !Double.isNaN(getPositionIntegrationMaxError());
   }

   /**
    * Gets the maximum position error to consider in the acceleration integration of this joint. How
    * this value is used is specific to the joint low level control.
    *
    * @return the maximum position error for acceleration integration for the joint.
    */
   double getPositionIntegrationMaxError();

   /**
    * Returns true if a maximum velocity error for acceleration integration was set for this joint.
    *
    * @see #getVelocityIntegrationMaxError()
    */
   default boolean hasVelocityIntegrationMaxError()
   {
      return !Double.isNaN(getVelocityIntegrationMaxError());
   }

   /**
    * Gets the maximum velocity error to consider in the acceleration integration of this joint. How
    * this value is used is specific to the joint low level control.
    *
    * @return the maximum velocity error for the joint.
    */
   double getVelocityIntegrationMaxError();

   /**
    * Returns true if a maximum position error for feedback was set for this joint.
    *
    * @see #getPositionFeedbackMaxError()
    */
   default boolean hasPositionFeedbackMaxError()
   {
      return !Double.isNaN(getPositionFeedbackMaxError());
   }

   /**
    * Gets the maximum position error to consider in the low level feedback control of this joint. How
    * this value is used is specific to the joint low level control.
    *
    * @return the maximum position error for the joint feedback control.
    */
   double getPositionFeedbackMaxError();

   /**
    * Convenience for clamping the desired position with {@link #getPositionFeedbackMaxError()} if it
    * is available.
    * 
    * @param currentPosition the current joint position that is to be used in the feedback controller.
    * @return the desired position clamped such that
    *         {@code |error| = Math.abs(getDesiredPosition() - currentPosition) <= getPositionFeedbackMaxError()}.
    */
   default double getClampedDesiredPosition(double currentPosition)
   {
      if (!hasDesiredPosition() || !hasPositionFeedbackMaxError())
         return getDesiredPosition();

      double error = getDesiredPosition() - currentPosition;

      if (Math.abs(error) > getPositionFeedbackMaxError())
      {
         double errorClamped = MathTools.clamp(error, getPositionFeedbackMaxError());
         return currentPosition + errorClamped;
      }
      else
      {
         return getDesiredPosition();
      }
   }

   /**
    * Returns true if a maximum velocity error for feedback was set for this joint.
    *
    * @see #getVelocityFeedbackMaxError()
    */
   default boolean hasVelocityFeedbackMaxError()
   {
      return !Double.isNaN(getVelocityFeedbackMaxError());
   }

   /**
    * Gets the maximum velocity error to consider in the low level feedback control of this joint. How
    * this value is used is specific to the joint low level control.
    *
    * @return the maximum velocity error for the joint feedback control.
    */
   double getVelocityFeedbackMaxError();

   /**
    * Convenience for clamping the desired velocity with {@link #getVelocityFeedbackMaxError()} if it
    * is available.
    * 
    * @param currentVelocity the current joint velocity that is to be used in the feedback controller.
    * @return the desired velocity clamped such that
    *         {@code |error| = Math.abs(getDesiredVelocity() - currentVelocity) <= getVelocityFeedbackMaxError()}.
    */
   default double getClampedDesiredVelocity(double currentVelocity)
   {
      if (!hasDesiredVelocity() || !hasVelocityFeedbackMaxError())
         return getDesiredVelocity();

      double error = getDesiredVelocity() - currentVelocity;

      if (Math.abs(error) > getVelocityFeedbackMaxError())
      {
         double errorClamped = MathTools.clamp(error, getVelocityFeedbackMaxError());
         return currentVelocity + errorClamped;
      }
      else
      {
         return getDesiredVelocity();
      }
   }

   /**
    * Copies the contents of this object to {@link JointDesiredOutputMessage}
    */
   default void copyToMessage(JointDesiredOutputMessage jointDesiredOutputMessage)
   {
      jointDesiredOutputMessage.setControlMode(hasControlMode() ? getControlMode().toByte() : (byte) 255);

      jointDesiredOutputMessage.setHasDesiredTorque(hasDesiredTorque());
      jointDesiredOutputMessage.setHasDesiredPosition(hasDesiredPosition());
      jointDesiredOutputMessage.setHasDesiredVelocity(hasDesiredVelocity());
      jointDesiredOutputMessage.setHasDesiredAcceleration(hasDesiredAcceleration());
      jointDesiredOutputMessage.setHasStiffness(hasStiffness());
      jointDesiredOutputMessage.setHasDamping(hasDamping());
      jointDesiredOutputMessage.setHasMasterGain(hasMasterGain());
      jointDesiredOutputMessage.setHasVelocityScaling(hasVelocityScaling());
      jointDesiredOutputMessage.setHasPositionIntegrationBreakFrequency(hasPositionIntegrationBreakFrequency());
      jointDesiredOutputMessage.setHasVelocityIntegrationBreakFrequency(hasVelocityIntegrationBreakFrequency());
      jointDesiredOutputMessage.setHasPositionIntegrationMaxError(hasPositionIntegrationMaxError());
      jointDesiredOutputMessage.setHasVelocityFeedbackMaxError(hasVelocityFeedbackMaxError());

      jointDesiredOutputMessage.setDesiredTorque(hasDesiredTorque() ? getDesiredTorque() : 0.0);
      jointDesiredOutputMessage.setDesiredPosition(hasDesiredPosition() ? getDesiredPosition() : 0.0);
      jointDesiredOutputMessage.setDesiredVelocity(hasDesiredVelocity() ? getDesiredVelocity() : 0.0);
      jointDesiredOutputMessage.setDesiredAcceleration(hasDesiredAcceleration() ? getDesiredAcceleration() : 0.0);
      jointDesiredOutputMessage.setStiffness(hasStiffness() ? getStiffness() : 0.0);
      jointDesiredOutputMessage.setDamping(hasDamping() ? getDamping() : 0.0);
      jointDesiredOutputMessage.setMasterGain(hasMasterGain() ? getMasterGain() : 0.0);
      jointDesiredOutputMessage.setVelocityScaling(hasVelocityScaling() ? getVelocityScaling() : 0.0);
      jointDesiredOutputMessage.setPositionIntegrationBreakFrequency(hasPositionIntegrationBreakFrequency() ? getPositionIntegrationBreakFrequency() : 0.0);
      jointDesiredOutputMessage.setVelocityIntegrationBreakFrequency(hasVelocityIntegrationBreakFrequency() ? getVelocityIntegrationBreakFrequency() : 0.0);
      jointDesiredOutputMessage.setPositionIntegrationMaxError(hasPositionIntegrationMaxError() ? getPositionIntegrationMaxError() : 0.0);
      jointDesiredOutputMessage.setVelocityFeedbackMaxError(hasVelocityFeedbackMaxError() ? getVelocityFeedbackMaxError() : 0.0);
   }

   default String getRepresentativeString()
   {
      String ret = "Joint Desired Output:\n";
      if (hasControlMode())
         ret += "controlMode = " + getControlMode() + "\n";
      if (hasLoadMode())
         ret += "loadMode = " + getLoadMode() + "\n";
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

   default String getShortRepresentativeString()
   {
      StringBuilder ret = new StringBuilder();
      if (hasControlMode())
         ret.append("mode= " + getControlMode());
      if (hasLoadMode())
      {
         if (ret.length() > 0)
            ret.append(", ");
         ret.append("load= " + getLoadMode());
      }
      if (hasDesiredTorque())
      {
         if (ret.length() > 0)
            ret.append(", ");
         ret.append("tau_d= " + getDesiredTorque());
      }
      if (hasDesiredPosition())
      {
         if (ret.length() > 0)
            ret.append(", ");
         ret.append("q_d= " + getDesiredPosition());
      }
      if (hasDesiredVelocity())
      {
         if (ret.length() > 0)
            ret.append(", ");
         ret.append("qd_d= " + getDesiredVelocity());
      }
      return ret.toString();
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
         if (getLoadMode() != other.getLoadMode())
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
         if (Double.compare(getVelocityIntegrationMaxError(), other.getVelocityIntegrationMaxError()) != 0)
            return false;
         if (Double.compare(getPositionIntegrationMaxError(), other.getPositionIntegrationMaxError()) != 0)
            return false;
         if (Double.compare(getVelocityFeedbackMaxError(), other.getVelocityFeedbackMaxError()) != 0)
            return false;
         if (Double.compare(getPositionFeedbackMaxError(), other.getPositionFeedbackMaxError()) != 0)
            return false;
         return true;
      }
   }
}