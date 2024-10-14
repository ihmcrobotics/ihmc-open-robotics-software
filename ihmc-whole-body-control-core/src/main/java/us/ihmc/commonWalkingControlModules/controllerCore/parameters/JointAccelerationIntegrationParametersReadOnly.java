package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

/**
 * Read-only interface that is implemented by a class holding the parameters necessary to perform
 * the integration of a joint desired acceleration into desired joint velocity and position.
 * <p>
 * The {@link WholeBodyInverseDynamicsSolver} can be requested to compute the desired position and
 * velocity for a list of joints by submitting a {@link JointAccelerationIntegrationCommand}.
 * </p>
 * <p>
 * When enabled for a joint, the {@code JointAccelerationIntegrationCalculator} will perform a
 * double step integration off of the desired joint acceleration to first compute the desired joint
 * velocity and finally the desired joint position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface JointAccelerationIntegrationParametersReadOnly
{
   /**
    * Whether the acceleration integration should be disabled for this joint regardless of whether
    * joint acceleration integration is enabled or not for this control session.
    * 
    * @return {@code true} to disable acceleration integration for this joint, {@code false} otherwise.
    */
   boolean getDisableAccelerationIntegration();

   /**
    * The break frequency for the position filter used in the acceleration integration. Increasing this
    * break frequency will cause the integration process to leak more heavily towards the current joint
    * position. Setting this to positive infinity will cause the integration to be turned off and the
    * desired joint position value to match the current joint position. Setting this to zero will do a
    * pure integration of the desired velocity.
    * <p>
    * The integrated position is computed as follows:</br>
    * q<sub>des</sub><sup>t</sup> = [(1 - &alpha;<sub>P</sub>) q<sub>cur</sub><sup>t</sup> +
    * &alpha;<sub>P</sub> q<sub>des</sub><sup>t - &Delta;t</sup>] + &Delta;t
    * qDot<sub>des</sub><sup>t</sup> </br>
    * Where &alpha;<sub>P</sub> is the decay rate computed from the integration timestep and the break
    * frequency defined here.
    * </p>
    * <p>
    * A low value for the break frequency will cause the joint to never settle by having a stick-slip
    * behavior around the "true" desired position the high-level controller is trying to achieve. It
    * can simply be increased until this undesirable effect disappears. The following default value can
    * be used as starting point for tuning a joint:<br>
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_POSITION_BREAK_FREQUENCY}.
    * </p>
    *
    * @return positionBreakFrequency the break frequency used to compute the desired position.
    */
   double getPositionBreakFrequency();

   /**
    * The break frequency for the velocity filter used in the acceleration integration. Increasing this
    * break frequency will cause the integration process to leak more heavily towards the reference
    * velocity. Setting this to positive infinity will cause the integration to be turned off and the
    * desired joint velocity value to be equal to the reference velocity. Setting this to zero will do
    * a pure integration of the desired acceleration.
    * <p>
    * The integrated position is computed as follows:<br>
    * qDot<sub>des</sub><sup>t</sup> = [(1 - &alpha;<sub>V</sub>) qDot<sub>ref</sub><sup>t</sup> +
    * &alpha;<sub>V</sub> qDot<sub>des</sub><sup>t - &Delta;t</sup>] + &Delta;t
    * qDDot<sub>des</sub><sup>t</sup> <br>
    * Where &alpha;<sub>V</sub> is the decay rate computed from the integration timestep and the break
    * frequency defined here.
    * <p>
    * Increasing the break frequency used to compute the desired velocity appears to be equivalent to
    * inserting damping to the joint. A high value will cause a loss of precision on the resulting
    * qDot<sub>des</sub> such that does impair the tracking that high-level controller is trying to
    * achieve. The following default value can be used as starting point for tuning a joint:<br>
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_VELOCITY_BREAK_FREQUENCY}.
    * </p>
    *
    * @return velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   double getVelocityBreakFrequency();

   /**
    * This is a safety parameter that is relevant to the tuning process for a joint. The default value
    * used by the calculator should be adequate in most situations: {@code maxPositionError} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only changed
    * once heavy testing has been performed on the robot knowing that the effects of this parameter may
    * show up only under specific circumstances.
    * </p>
    *
    * @return maxPositionError limits the gap between the desired joint position and the actual joint
    *         position.
    */
   double getMaxPositionError();

   /**
    * This is a safety parameter that is relevant to the tuning process for a joint. The default value
    * used by the calculator should be adequate in most situations: {@code maxVelocity} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_VELOCITY_ERROR}.
    * <p>
    * The maximum velocity error parameter is used to limit the gap between the desired velocity
    * computed and the reference joint velocity, see {@link #getVelocityReferenceAlpha()}. It can be
    * increased once the acceleration integration is proven to be working properly on a specific robot
    * to allow the joint to reach higher velocities.
    * </p>
    *
    * @return maxVelocityError limits the gap between the desired joint velocity and the reference
    *         joint velocity.
    */
   double getMaxVelocityError();

   /**
    * Factor used for computing the reference velocity towards which the integrator will bleed the
    * desired velocity.
    * <p>
    * The value should be in [0, 1]. A value of 0 indicates that the reference velocity should be zero
    * while a value of 1 indicates that the reference velocity should be the measured joint velocity.
    * </p>
    * <p>
    * A lower value for this parameter will increase stability as the computed desired velocity will
    * contain less noise. However, a low value also makes it harder for the desired velocity to reach
    * high values.
    * </p>
    * 
    * @return velocityReferenceAlpha the ratio of measured velocity to be used as reference.
    */
   double getVelocityReferenceAlpha();

   /**
    * This is to configure how the desired velocity is to be reset upon receiving
    * {@link JointDesiredOutputReadOnly#peekResetIntegratorsRequest()} in
    * {@link JointAccelerationIntegrationCalculator}.
    * <ul>
    * <li>{@link JointVelocityIntegratorResetMode#CURRENT_VELOCITY} (default behavior), the desired
    * velocity is reset to the current joint velocity.
    * <li>{@link JointVelocityIntegratorResetMode#ZERO_VELOCITY} when {@code true}, the desired
    * velocity is reset to zero.
    * <li>{@link JointVelocityIntegratorResetMode#REFERENCE_VELOCITY} when {@code true}, the desired
    * velocity is reset to the reference velocity, see {@link #getVelocityReferenceAlpha()}.
    * </ul>
    * 
    * @return specifies the integrator's behavior for when resetting the desired velocity. Default
    *         value is {@link JointVelocityIntegratorResetMode#CURRENT_VELOCITY}.
    */
   JointVelocityIntegratorResetMode getVelocityResetMode();
}
