package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.euclid.interfaces.Settable;

public class JointAccelerationIntegrationParameters implements JointAccelerationIntegrationParametersReadOnly, Settable<JointAccelerationIntegrationParameters>
{
   private double positionBreakFrequency;
   private double velocityBreakFrequency;
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
    * Resets the values for {@code positionBreakFrequency} and {@code velocityBreakFrequency} to
    * {@link Double#NaN} notifying the {@link JointAccelerationIntegrationCalculator} to use its
    * default values.
    */
   public void resetAlphas()
   {
      positionBreakFrequency = Double.NaN;
      velocityBreakFrequency = Double.NaN;
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
   @Override
   public void set(JointAccelerationIntegrationParameters other)
   {
      set((JointAccelerationIntegrationParametersReadOnly) other);
   }

   /**
    * Sets the parameters of this to the values of the parameters of other.
    *
    * @param other the other set of parameters. Not modified.
    */
   public void set(JointAccelerationIntegrationParametersReadOnly other)
   {
      positionBreakFrequency = other.getPositionBreakFrequency();
      velocityBreakFrequency = other.getVelocityBreakFrequency();
      maxPositionError = other.getMaxPositionError();
      maxVelocity = other.getMaxVelocity();
   }

   /**
    * Sets the two break frequencies for the leak rates in the acceleration integration.
    * <p>
    * For the usage of these parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getPositionBreakFrequency()}<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getVelocityBreakFrequency()}
    * </p>
    *
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    * @param velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   public void setBreakFrequencies(double positionBreakFrequency, double velocityBreakFrequency)
   {
      this.positionBreakFrequency = positionBreakFrequency;
      this.velocityBreakFrequency = velocityBreakFrequency;
   }

   /**
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values for the
    * {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a joint.
    * The default values used by the calculator should be adequate in most situation.
    * </p>
    * <p>
    * The maximum velocity parameter is used to saturate the value of the desired velocity computed. If
    * not specified otherwise, {@code maxVelocity} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_VELOCITY}. It can be increased once the
    * acceleration integration is proven to be working properly on a specific robot to allow the joint
    * to reach higher velocities.
    * </p>
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only changed
    * once heavy testing has been performed on the robot knowing that the effects of this parameter may
    * show up only under specific circumstances. If not specified otherwise {@code maxPositionError} =
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
    * For the usage of this parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getPositionBreakFrequency()}
    *
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    */
   public void setPositionBreakFrequency(double positionBreakFrequency)
   {
      this.positionBreakFrequency = positionBreakFrequency;
   }

   /**
    * For the usage of this parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getVelocityBreakFrequency()}
    *
    * @param velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   public void setVelocityBreakFrequency(double velocityBreakFrequency)
   {
      this.velocityBreakFrequency = velocityBreakFrequency;
   }

   /**
    * Sets the safety parameter that limits the position error between the actual joint position and
    * the integrated desired.
    * 
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
    * 
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
   public double getPositionBreakFrequency()
   {
      return positionBreakFrequency;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityBreakFrequency()
   {
      return velocityBreakFrequency;
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

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointAccelerationIntegrationParametersReadOnly)
      {
         JointAccelerationIntegrationParametersReadOnly other = (JointAccelerationIntegrationParametersReadOnly) object;
         if (positionBreakFrequency != other.getPositionBreakFrequency())
            return false;
         if (velocityBreakFrequency != other.getVelocityBreakFrequency())
            return false;
         if (maxPositionError != other.getMaxPositionError())
            return false;
         if (maxVelocity != other.getMaxVelocity())
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": position break frequency: " + positionBreakFrequency + ", velocity break frequency: " + velocityBreakFrequency
            + ", max position error: " + maxPositionError + ", max velocity: " + maxVelocity;
   }
}
