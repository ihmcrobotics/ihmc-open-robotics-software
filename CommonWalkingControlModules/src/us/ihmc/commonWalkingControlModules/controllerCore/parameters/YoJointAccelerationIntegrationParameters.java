package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointAccelerationIntegrationParameters implements JointAccelerationIntegrationParametersReadOnly
{
   private final YoDouble alphaPosition;
   private final YoDouble alphaVelocity;
   private final YoDouble maxPositionError;
   private final YoDouble maxVelocity;

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * The parameters are initialized with the default values of
    * {@link JointAccelerationIntegrationCalculator}.
    * </p>
    * 
    * @param namePrefix the {@code String} to be prepended to each {@code YoVariable} of this class.
    * @param registry the registry to which the {@code YoVariable}s of this class are registered to.
    */
   public YoJointAccelerationIntegrationParameters(String namePrefix, YoVariableRegistry registry)
   {
      alphaPosition = new YoDouble(namePrefix + "AlphaPosition", registry);
      alphaVelocity = new YoDouble(namePrefix + "AlphaVelocity", registry);
      maxPositionError = new YoDouble(namePrefix + "MaxPositionError", registry);
      maxVelocity = new YoDouble(namePrefix + "MaxVelocity", registry);

      reset();
   }

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * The parameters are initialized to the given values.
    * </p>
    * 
    * @param namePrefix the {@code String} to be prepended to each {@code YoVariable} of this class.
    * @param alphaPosition the leak ratio used to compute the desired position, see {@link #getAlphaPosition()}.
    * @param alphaVelocity the leak ratio used to compute the desired velocity, see {@link #getAlphaVelocity()}.
    * @param maxPositionError the maximum position error used to saturate the desired position, see {@link #getMaxPositionError()}.
    * @param maxVelocity the maximum desired velocity, see {@link #getMaxVelocity()}.
    * @param registry the registry to which the {@code YoVariable}s of this class are registered to.
    */
   public YoJointAccelerationIntegrationParameters(String namePrefix, double alphaPosition, double alphaVelocity, double maxPositionError, double maxVelocity,
                                                   YoVariableRegistry registry)
   {
      this(namePrefix, registry);
      setAlphas(alphaPosition, alphaVelocity);
      setMaxima(maxPositionError, maxVelocity);
   }

   /**
    * Resets the values for this class parameters to the default values from
    * {@link JointAccelerationIntegrationCalculator}.
    */
   public void reset()
   {
      resetAlphas();
      resetMaxima();
   }

   /**
    * Resets the values for {@code alphaPosition} and {@code alphaVelocity} to the default values
    * from {@link JointAccelerationIntegrationCalculator}.
    */
   public void resetAlphas()
   {
      alphaPosition.set(JointAccelerationIntegrationCalculator.DEFAULT_ALPHA_POSITION);
      alphaVelocity.set(JointAccelerationIntegrationCalculator.DEFAULT_ALPHA_VELOCITY);
   }

   /**
    * Resets the values for {@code maxPositionError} and {@code maxVelocity} to the default values
    * from {@link JointAccelerationIntegrationCalculator}.
    */
   public void resetMaxima()
   {
      maxPositionError.set(JointAccelerationIntegrationCalculator.DEFAULT_MAX_POSITION_ERROR);
      maxVelocity.set(JointAccelerationIntegrationCalculator.DEFAULT_MAX_VELOCITY);
   }

   /**
    * The two alpha parameters are used as leak ratios for each integration: <br>
    * Desired velocity:<br>
    * qDot<sub>des</sub><sup>t</sup> = &alpha;<sub>V</sub> qDot<sub>des</sub><sup>t - &Delta;t</sup>
    * + &Delta;t qDDot<sub>des</sub><sup>t</sup> <br>
    * Desired position:<br>
    * q<sub>des</sub><sup>t</sup> = (1 - &alpha;<sub>P</sub>) q<sub>cur</sub><sup>t</sup> +
    * &alpha;<sub>P</sub> (q<sub>des</sub><sup>t - &Delta;t</sup> + &Delta;t
    * qDot<sub>des</sub><sup>t</sup>)
    * </p>
    * <p>
    * Both leak ratios have to be &in; [0, 1].
    * </p>
    * <p>
    * Decreasing the leak ratio &alpha;<sub>V</sub> used to compute the desired velocity appears to
    * be equivalent to inserting damping to the joint. A low value will cause a loss of precision on
    * the resulting q<sub>des</sub> such it does impair the tracking that high-level controller is
    * performing. If not specified otherwise, &alpha;<sub>V</sub> =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_ALPHA_VELOCITY}.
    * </p>
    * <p>
    * A high value for the leak ratio &alpha;<sub>P</sup> used to compute the desired position will
    * cause the joint to never settle by having stick-slip behavior around the "true" desired
    * position the high-level controller is trying to achieve. It can simply be downtuned until this
    * undesirable effect disappear. If not specified otherwise, &alpha;<sub>P</sup> =
    * {@link JointAccelerationIntegrationCalculator#DEFAUTL_ALPHA_POSITION}.
    * </p>
    * 
    * @param alphaPosition the leak ratio &alpha;<sub>P</sup> used to compute the desired position.
    * @param alphaVelocity the leak ratio &alpha;<sub>V</sup> used to compute the desired velocity.
    */
   public void setAlphas(double alphaPosition, double alphaVelocity)
   {
      this.alphaPosition.set(alphaPosition);
      this.alphaVelocity.set(alphaVelocity);
   }

   /**
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a
    * joint. The default values used by the calculator should be adequate in most situation.
    * </p>
    * <p>
    * The maximum velocity parameter is used to saturate the value of the desired velocity computed.
    * If not specified otherwise, {@code maxVelocity} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_VELOCITY}. It can be increased once
    * the acceleration integration is proven to be working properly on a specific robot to allow the
    * joint to reach higher velocities.
    * </p>
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only
    * changed once heavy testing has been performed on the robot knowing that the effects of this
    * parameter may show up only under specific circumstances. If not specified otherwise
    * {@code maxPositionError} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * </p>
    * 
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    * @param maxVelocity limits the maximum value of the desired joint velocity.
    */
   public void setMaxima(double maxPositionError, double maxVelocity)
   {
      this.maxPositionError.set(maxPositionError);
      this.maxVelocity.set(maxVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaPosition()
   {
      return alphaPosition.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaVelocity()
   {
      return alphaVelocity.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxPositionError()
   {
      return maxPositionError.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxVelocity()
   {
      return maxVelocity.getDoubleValue();
   }
}
