package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;

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
 *
 */
public interface JointAccelerationIntegrationParametersReadOnly
{
   /**
    * The alpha position parameter, referred below as &alpha;<sub>P</sub>, is used as a leak ratio
    * for the second step of the acceleration integration: <br>
    * q<sub>des</sub><sup>t</sup> = (1 - &alpha;<sub>P</sub>) q<sub>cur</sub><sup>t</sup> +
    * &alpha;<sub>P</sub> (q<sub>des</sub><sup>t - &Delta;t</sup> + &Delta;t
    * qDot<sub>des</sub><sup>t</sup>)<br>
    * where &Delta;t is the duration of a control tick, q<sub>des</sub><sup>t</sup> is the newly
    * computed desired joint position, q<sub>des</sub><sup>t - &Delta;t</sup> is the previous value
    * of the desired joint position, q<sub>cur</sub><sup>t</sup> is the current joint position, and
    * qDot<sub>des</sub><sup>t</sup> is the desired joint velocity.
    * <p>
    * The leak ratio &alpha;<sub>P</sub> has to be &in; [0, 1].
    * </p>
    * <p>
    * A high value for the leak ratio &alpha;<sub>P</sub> will cause the joint to never settle by
    * having a stick-slip behavior around the "true" desired position the high-level controller is
    * trying to achieve. It can simply be downtuned until this undesirable effect disappear. The
    * following default value can be used as starting point for tuning a joint: &alpha;<sub>P</sub>
    * = {@link JointAccelerationIntegrationCalculator#DEFAULT_ALPHA_POSITION}.
    * </p>
    *
    * @return alphaPosition the leak ratio &alpha;<sub>P</sup> used to compute the desired position.
    */
   double getAlphaPosition();

   /**
    * The alpha velocity parameter, referred below as &alpha;<sub>V</sub>, is used as a leak ratio
    * for the first step of the acceleration integration: <br>
    * qDot<sub>des</sub><sup>t</sup> = &alpha;<sub>V</sub> qDot<sub>des</sub><sup>t - &Delta;t</sup>
    * + &Delta;t qDDot<sub>des</sub><sup>t</sup> <br>
    * where &Delta;t is the duration of a control tick, qDot<sub>des</sub><sup>t</sup> is the newly
    * computed desired joint velocity, qDot<sub>des</sub><sup>t - &Delta;t</sup> is the previous
    * value of the desired joint velocity, and qDDot<sub>des</sub><sup>t</sup> is the desired joint
    * acceleration.
    * <p>
    * The leak ratio &alpha;<sub>V</sub> has to be &in; [0, 1].
    * </p>
    * <p>
    * Decreasing the leak ratio &alpha;<sub>V</sub> used to compute the desired velocity appears to
    * be equivalent to inserting damping to the joint. A low value will cause a loss of precision on
    * the resulting q<sub>des</sub> such it does impair the tracking that high-level controller is
    * performing. The following default value can be used as starting point for tuning a joint:
    * &alpha;<sub>V</sub> = {@link JointAccelerationIntegrationCalculator#DEFAULT_ALPHA_VELOCITY}.
    * </p>
    *
    * @return alphaVelocity the leak ratio &alpha;<sub>V</sup> used to compute the desired velocity.
    */
   double getAlphaVelocity();

   /**
    * This is a safety parameter that is relevant to the tuning process for a joint. The default
    * value used by the calculator should be adequate in most situations: {@code maxPositionError} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
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
    * value used by the calculator should be adequate in most situations: {@code maxVelocity} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_VELOCITY}.
    * <p>
    * The maximum velocity parameter is used to saturate the value of the desired velocity computed.
    * It can be increased once the acceleration integration is proven to be working properly on a
    * specific robot to allow the joint to reach higher velocities.
    * </p>
    *
    * @return maxVelocity limits the maximum value of the desired joint velocity.
    */
   double getMaxVelocity();
}
