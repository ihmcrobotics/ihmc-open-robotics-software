package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointVelocityIntegrationCalculator;


public class JointVelocityIntegrationParameters implements JointVelocityIntegrationParametersReadOnly
{
   private double positionBreakFrequency;
   private double accelerationBreakFrequency;
   private double maxPositionError;
   private double maxAcceleration;

   /**
    * Creates a new sets of parameters for velocity integration.
    * <p>
    * The parameters are initialized to {@link Double#NaN} such notifying the
    * {@link JointVelocityIntegrationCalculator} to use its default parameters.
    * </p>
    */
   // This empty constructor is notably needed to be able to use RecyclingArrayList<>(Class)
   public JointVelocityIntegrationParameters()
   {
      reset();
   }

   /**
    * Resets all the parameters to the default values from
    * {@link JointVelocityIntegrationCalculator}.
    */
   public void reset()
   {
      resetAlphas();
      resetMaxima();
   }

   /**
    * Resets the values for {@code positionBreakFrequency} and {@code accelerationBreakFrequency} to {@link Double#NaN}
    * notifying the {@link JointVelocityIntegrationCalculator} to use its default values.
    */
   public void resetAlphas()
   {
      positionBreakFrequency = Double.NaN;
      accelerationBreakFrequency = Double.NaN;
   }

   /**
    * Resets the values for {@code maxPositionError} and {@code maxAcceleration} to {@link Double#NaN}
    * notifying the {@link JointVelocityIntegrationCalculator} to use its default values.
    */
   public void resetMaxima()
   {
      maxPositionError = Double.NaN;
      maxAcceleration = Double.NaN;
   }

   /**
    * Sets the parameters of this to the values of the parameters of other.
    *
    * @param other the other set of parameters. Not modified.
    */
   public void set(JointVelocityIntegrationParametersReadOnly other)
   {
      positionBreakFrequency = other.getPositionBreakFrequency();
      accelerationBreakFrequency = other.getAccelerationBreakFrequency();
      maxPositionError = other.getMaxPositionError();
      maxAcceleration = other.getMaxAcceleration();
   }

   /**
    * Sets the two break frequencies for the leak rate in the velocity integration and alpha
    * filter for velocity differentiation
    * <p>
    * For the usage of these parameters see<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getPositionBreakFrequency()}<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getAccelerationBreakFrequency()}
    * </p>
    *
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    * @param accelerationBreakFrequency the break frequency used to compute the desired acceleration.
    */
   public void setBreakFrequencies(double positionBreakFrequency, double accelerationBreakFrequency)
   {
      this.positionBreakFrequency = positionBreakFrequency;
      this.accelerationBreakFrequency = accelerationBreakFrequency;
   }

   /**
    * Provides to the {@link JointVelocityIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a
    * joint. The default values used by the calculator should be adequate in most situation.
    * </p>
    * <p>
    * The maximum acceleration parameter is used to saturate the value of the desired acceleration computed.
    * If not specified otherwise, {@code maxAcceleration} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_ACCELERATION}. It can be increased once
    * the velocity differentiation is proven to be working properly on a specific robot to allow the
    * joint to reach higher accelerations.
    * </p>
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only
    * changed once heavy testing has been performed on the robot knowing that the effects of this
    * parameter may show up only under specific circumstances. If not specified otherwise
    * {@code maxPositionError} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * </p>
    *
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    * @param maxAcceleration limits the maximum value of the desired joint acceleration.
    */
   public void setMaxima(double maxPositionError, double maxAcceleration)
   {
      this.maxPositionError = maxPositionError;
      this.maxAcceleration = maxAcceleration;
   }

   /**
    * For the usage of this parameters see<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getPositionBreakFrequency()}
    *
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    */
   public void setPositionBreakFrequency(double positionBreakFrequency)
   {
      this.positionBreakFrequency = positionBreakFrequency;
   }

   /**
    * For the usage of this parameters see<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getAccelerationBreakFrequency()}
    *
    * @param accelerationBreakFrequency the break frequency used to compute the desired acceleration.
    */
   public void setAccelerationBreakFrequency(double accelerationBreakFrequency)
   {
      this.accelerationBreakFrequency = accelerationBreakFrequency;
   }

   /**
    * Sets the safety parameter that limits the position error between the actual joint position
    * and the integrated desired.
    * @see JointVelocityIntegrationParameters#setMaxima(double, double)
    *
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    */
   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError = maxPositionError;
   }

   /**
    * Sets the safety parameter that limits the differentiated acceleration.
    * @see JointVelocityIntegrationParameters#setMaxima(double, double)
    *
    * @param maxAcceleration limits the maximum value of the desired joint acceleration.
    */
   public void setMaxAcceleration(double maxAcceleration)
   {
      this.maxAcceleration = maxAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public double getPositionBreakFrequency()
   {
      return positionBreakFrequency;
   }

   /** {@inheritDoc} */
   @Override
   public double getAccelerationBreakFrequency()
   {
      return accelerationBreakFrequency;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxPositionError()
   {
      return maxPositionError;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAcceleration()
   {
      return maxAcceleration;
   }
}
