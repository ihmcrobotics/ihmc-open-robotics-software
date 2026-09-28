package us.ihmc.avatar.scs2;

/**
 * Receives a joint's low-level command instead of the torque {@link SCS2OutputWriter} would
 * otherwise compute from it.
 *
 * <p>SCS2's engine-facing contract is a scalar effort per joint: a controller writes it into
 * {@code ControllerOutput} and the engine reads it off the joint. That is the right contract for an
 * engine with no actuator model, but it discards the gains and setpoints on the way, so the
 * impedance loop ends up closing at the controller's rate rather than the plant's. An engine that
 * can close the loop itself takes the command whole through one of these.
 *
 * <p>Returning false leaves the writer to compute and write a torque as usual, which is how joints
 * the engine cannot drive keep working.
 *
 * @see SCS2OutputWriter#setJointCommand(JointCommand)
 */
public interface JointCommand
{
   /**
    * @param jointName the simulated joint the command is for.
    * @param feedforwardTorque the controller's desired torque, after any effort corruption.
    * @param desiredPosition the position setpoint, already clamped to the feedback error limit.
    * @param desiredVelocity the velocity setpoint, already scaled and clamped.
    * @param stiffness position feedback gain.
    * @param damping velocity feedback gain.
    * @return true if the command was consumed; false to leave the writer to write a torque.
    */
   boolean setJointCommand(String jointName,
                           double feedforwardTorque,
                           double desiredPosition,
                           double desiredVelocity,
                           double stiffness,
                           double damping);
}
