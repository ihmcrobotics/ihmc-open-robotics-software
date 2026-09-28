package us.ihmc.avatar.scs2;

/**
 * Receives a joint's low-level command instead of the torque {@link SCS2OutputWriter} would
 * otherwise compute from it.
 *
 * <p>SCS2's engine-facing contract is a scalar effort per joint: a controller writes it into
 * {@code ControllerOutput} and the engine reads it off the joint. That is the right contract for an
 * engine that has no actuator model, but it throws away the gains and setpoints on the way, so the
 * impedance loop ends up closing at the controller's rate instead of the plant's. A physics engine
 * that can close the loop itself -- MuJoCo, through its actuators -- supplies one of these to get
 * the command intact.
 *
 * <p>The sink is offered the command at the point {@link SCS2OutputWriter} would have written the
 * effort, which is after corruption, velocity scaling and the feedback-error clamps have been
 * applied. That matters: it is also downstream of {@code InterpolatedSCS2OutputWriter} and the
 * low-level output processor, so a sink placed here sees interpolated, processed desireds rather
 * than the raw ones.
 *
 * <p>Returning false leaves the writer to compute and write the effort as usual, which is how
 * joints the engine cannot drive keep working.
 */
public interface JointCommandSink
{
   /**
    * @param jointName the simulated joint the command is for.
    * @param feedforwardTorque the controller's desired torque, after any effort corruption.
    * @param desiredPosition the position setpoint, already clamped to the feedback error limit.
    * @param desiredVelocity the velocity setpoint, already scaled and clamped.
    * @param stiffness position feedback gain.
    * @param damping velocity feedback gain.
    * @return true if the command was consumed; false to fall back to writing a torque.
    */
   boolean setJointCommand(String jointName,
                           double feedforwardTorque,
                           double desiredPosition,
                           double desiredVelocity,
                           double stiffness,
                           double damping);
}
