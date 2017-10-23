package us.ihmc.sensorProcessing.outputData;

/**
 * An interface for a data holder that is used to communicate desired joint behavior
 * (setpoints and controller properties) from a whole body controller to joint level
 * controllers.
 */
public interface JointDesiredOutputReadOnly
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
}