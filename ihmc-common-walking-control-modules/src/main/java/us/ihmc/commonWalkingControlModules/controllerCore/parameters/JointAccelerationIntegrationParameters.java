package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;

public class JointAccelerationIntegrationParameters implements JointAccelerationIntegrationParametersReadOnly
{
   private double alphaPosition;
   private double alphaVelocity;
   private double maxPositionError;
   private double maxVelocity;

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * The parameters are initialized to {@link Double#NaN} such notifying the
    * {@link JointAccelerationIntegrationCalculator} to use its default parameters.
    * </p>
    */
   // This empty constructor is notably needed to be able to use RecyclingArrayList<>(Class)
   public JointAccelerationIntegrationParameters()
   {
      reset();
   }

   /**
    * Resets all the parameters to the default values from
    * {@link JointAccelerationIntegrationCalculator}.
    */
   public void reset()
   {
      resetAlphas();
      resetMaxima();
   }

   /**
    * Resets the values for {@code alphaPosition} and {@code alphaVelocity} to {@link Double#NaN}
    * notifying the {@link JointAccelerationIntegrationCalculator} to use its default values.
    */
   public void resetAlphas()
   {
      alphaPosition = Double.NaN;
      alphaVelocity = Double.NaN;
   }

   /**
    * Resets the values for {@code maxPositionError} and {@code maxVelocity} to {@link Double#NaN}
    * notifying the {@link JointAccelerationIntegrationCalculator} to use its default values.
    */
   public void resetMaxima()
   {
      maxPositionError = Double.NaN;
      maxVelocity = Double.NaN;
   }

   /**
    * Sets the parameters of this to the values of the parameters of other.
    * 
    * @param other the other set of parameters. Not modified.
    */
   public void set(JointAccelerationIntegrationParametersReadOnly other)
   {
      alphaPosition = other.getAlphaPosition();
      alphaVelocity = other.getAlphaVelocity();
      maxPositionError = other.getMaxPositionError();
      maxVelocity = other.getMaxVelocity();
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
      this.alphaPosition = alphaPosition;
      this.alphaVelocity = alphaVelocity;
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
      this.maxPositionError = maxPositionError;
      this.maxVelocity = maxVelocity;
   }

   /**
    * Sets the leak ratio for the integration of the joint position.
    * @see JointAccelerationIntegrationParameters#setAlphas(double, double)
    *
    * @param alphaPosition the leak ratio &alpha;<sub>P</sup> used to compute the desired position.
    */
   public void setAlphaPosition(double alphaPosition)
   {
      this.alphaPosition = alphaPosition;
   }

   /**
    * Sets the leak ratio for the integration of the joint velocity.
    * @see JointAccelerationIntegrationParameters#setAlphas(double, double)
    *
    * @param alphaVelocity the leak ratio &alpha;<sub>V</sup> used to compute the desired velocity.
    */
   public void setAlphaVelocity(double alphaVelocity)
   {
      this.alphaVelocity = alphaVelocity;
   }

   /**
    * Sets the safety parameter that limits the position error between the actual joint position
    * and the integrated desired.
    * @see JointAccelerationIntegrationParameters#setMaxima(double, double)
    *
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    */
   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError = maxPositionError;
   }

   /**
    * Sets the safety parameter that limits the integrated velocity.
    * @see JointAccelerationIntegrationParameters#setMaxima(double, double)
    *
    * @param maxVelocity limits the maximum value of the desired joint velocity.
    */
   public void setMaxVelocity(double maxVelocity)
   {
      this.maxVelocity = maxVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaPosition()
   {
      return alphaPosition;
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaVelocity()
   {
      return alphaVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxPositionError()
   {
      return maxPositionError;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxVelocity()
   {
      return maxVelocity;
   }
}
