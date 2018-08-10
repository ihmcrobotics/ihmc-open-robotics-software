package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointVelocityIntegrationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointVelocityIntegrationCalculator;

/**
 * Read-only interface that is implemented by a class holding the parameters necessary to perform
 * the integration of a joint desired velocity into position and differentiation into acceleration
 * <p>
 * The {@link WholeBodyInverseKinematicsSolver} can be requested to compute the desired position and
 * acceleration for a list of joints by submitting a {@link JointVelocityIntegrationCommand}.
 * </p>
 * <p>
 * When enabled for a joint, the {@code JointVelocityIntegrationCalculator} will perform a
 * single step integration off of the desired joint velocity to compute the desired joint
 * position and then a single step numerical differentiation to compute the desired joint
 * acceleration.
 * </p>
 *
 * @author Robert Griffin
 *
 */
public interface JointVelocityIntegrationParametersReadOnly
{
   /**
    * The break frequency for the position filter used in the velocity integration. Increasing
    * this break frequency will cause the integration process to leak more heavily towards the
    * current joint position. Setting this to positive infinity will cause the integration to be
    * turned off and the desired joint position value to match the current joint position. Setting
    * this to zero will do a pure integration of the desired velocity.
    * <p>
    * The integrated position is computed as follows:</br>
    * q<sub>des</sub><sup>t</sup> = [(1 - &alpha;<sub>P</sub>) q<sub>cur</sub><sup>t</sup> +
    * &alpha;<sub>P</sub> q<sub>des</sub><sup>t - &Delta;t</sup>] + &Delta;t
    * qDot<sub>des</sub><sup>t</sup> </br>
    * Where &alpha;<sub>P</sub> is the decay rate computed from the integration timestep and the
    * break frequency defined here.
    * </p>
    * <p>
    * A low value for the break frequency will cause the joint to never settle by
    * having a stick-slip behavior around the "true" desired position the high-level controller is
    * trying to achieve. It can simply be increased until this undesirable effect disappears. The
    * following default value can be used as starting point for tuning a joint:<br>
    * {@link JointVelocityIntegrationCalculator#DEFAULT_POSITION_BREAK_FREQUENCY}.
    * </p>
    *
    * @return positionBreakFrequency the break frequency used to compute the desired position.
    */
   double getPositionBreakFrequency();

   /**
    * The break frequency for the acceleration filter used in the velocity differentiation. Increasing
    * this break frequency will cause the differentiation process to more heavily favor the new
    * signal. Setting this to positive infinity will use only the current numerical derivative. Setting
    * this to zero will be completely filtered, and not update at all.
    * <p>
    * The differentiated acceleration is computed as follows:<br>
    * qDDot<sub>des</sub><sup>t</sup> = [&alpha;<sub>a</sub> qDDot<sub>des</sub><sup>t - &Delta;t</sup>]
    * + [(1.0 - &alpha;<sub>a</sub>) (qDot<sub>des</sub> <sup>t</sup> - qDot<sub>des</sub><sup>t - &Delta;t</sup>) / &Delta;t] <br>
    * Where &alpha;<sub>a</sub> is the decay rate computed from the integration timestep and the
    * break frequency defined here.
    * <p>
    * The following default value can be used as starting point for tuning a joint:<br>
    * {@link JointVelocityIntegrationCalculator#DEFAULT_ACCELERATION_BREAK_FREQUENCY}.
    * </p>
    *
    * @return velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   double getAccelerationBreakFrequency();

   /**
    * This is a safety parameter that is relevant to the tuning process for a joint. The default
    * value used by the calculator should be adequate in most situations: {@code maxPositionError} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only
    * changed once heavy testing has been performed on the robot knowing that the effects of this
    * parameter may show up only under specific circumstances.
    * </p>
    *
    * @return maxPositionError limits the gap between the desired joint position and the actual
    *         joint position.
    */
   double getMaxPositionError();

   /**
    * This is a safety parameter that is relevant to the tuning process for a joint. The default
    * value used by the calculator should be adequate in most situations: {@code maxAcceleration} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_ACCELERATION}.
    * <p>
    * The maximum acceleration parameter is used to saturate the value of the desired acceleration computed.
    * It can be increased once the velocity differentiation is proven to be working properly on a
    * specific robot to allow the joint to reach higher acceleration.
    * </p>
    *
    * @return maxAcceleration limits the maximum value of the desired joint acceleration.
    */
   double getMaxAcceleration();
}
