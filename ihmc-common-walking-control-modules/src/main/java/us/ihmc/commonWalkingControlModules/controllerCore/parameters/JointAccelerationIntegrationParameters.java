package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.euclid.interfaces.Settable;

public class JointAccelerationIntegrationParameters implements JointAccelerationIntegrationParametersReadOnly, Settable<JointAccelerationIntegrationParameters>
{
   private boolean disableAccelerationIntegration;
   private double positionBreakFrequency;
   private double velocityBreakFrequency;
   private double maxPositionError;
   private double maxVelocityError;
   private double velocityReferenceAlpha;
   private JointVelocityIntegratorResetMode velocityResetMode;

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
      disableAccelerationIntegration = false;
      resetAlphas();
      resetMaxima();
      velocityResetMode = null;
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
      velocityReferenceAlpha = Double.NaN;
   }

   /**
    * Resets the values for {@code maxPositionError} and {@code maxVelocityError} to {@link Double#NaN}
    * notifying the {@link JointAccelerationIntegrationCalculator} to use its default values.
    */
   public void resetMaxima()
   {
      maxPositionError = Double.NaN;
      maxVelocityError = Double.NaN;
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
      disableAccelerationIntegration = other.getDisableAccelerationIntegration();
      positionBreakFrequency = other.getPositionBreakFrequency();
      velocityBreakFrequency = other.getVelocityBreakFrequency();
      maxPositionError = other.getMaxPositionError();
      maxVelocityError = other.getMaxVelocityError();
      velocityReferenceAlpha = other.getVelocityReferenceAlpha();
      velocityResetMode = other.getVelocityResetMode();
   }

   /**
    * Sets whether to disable acceleration integration for this joint.
    * 
    * @param disableAccelerationIntegration {@code true} to disable acceleration for this joint.
    *                                       Default value {@code false}.
    */
   public void setDisableAccelerationIntegration(boolean disableAccelerationIntegration)
   {
      this.disableAccelerationIntegration = disableAccelerationIntegration;
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
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a joint.
    * The default values used by the calculator should be adequate in most situation.
    * </p>
    *
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *                         position.
    * @param maxVelocityError limits the gap between the desired joint velocity and the reference joint
    *                         velocity.
    * @see JointAccelerationIntegrationParametersReadOnly#getMaxPositionError()
    * @see JointAccelerationIntegrationParametersReadOnly#getMaxVelocityError()
    * @see JointAccelerationIntegrationParametersReadOnly#getVelocityReferenceAlpha()
    */
   public void setMaxima(double maxPositionError, double maxVelocityError)
   {
      this.maxPositionError = maxPositionError;
      this.maxVelocityError = maxVelocityError;
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
    * For the usage of this parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getVelocityResetMode()}
    * 
    * @param velocityResetMode the integrator's behavior for when resetting the desired velocity.
    *                          Default value is
    *                          {@link JointVelocityIntegratorResetMode.CURRENT_VELOCITY}.
    */
   public void setVelocityResetMode(JointVelocityIntegratorResetMode velocityResetMode)
   {
      this.velocityResetMode = velocityResetMode;
   }

   /**
    * Sets the safety parameter that limits the position error between the actual joint position and
    * the integrated desired.
    * 
    * @see JointAccelerationIntegrationParameters#setMaxima(double, double)
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *                         position.
    */
   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError = maxPositionError;
   }

   /**
    * Sets the safety parameter that limits the integrated velocity.
    * 
    * @see JointAccelerationIntegrationParameters#setMaxima(double, double)
    * @param maxVelocityError limits the gap between the desired joint velocity and the reference joint
    *                         velocity.
    */
   public void setMaxVelocityError(double maxVelocityError)
   {
      this.maxVelocityError = maxVelocityError;
   }

   /**
    * For the usage of this parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getVelocityReferenceAlpha()}
    *
    * @param velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   public void setVelocityReferenceAlpha(double velocityReferenceAlpha)
   {
      this.velocityReferenceAlpha = velocityReferenceAlpha;
   }

   @Override
   public boolean getDisableAccelerationIntegration()
   {
      return disableAccelerationIntegration;
   }

   @Override
   public JointVelocityIntegratorResetMode getVelocityResetMode()
   {
      return velocityResetMode;
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
   public double getMaxVelocityError()
   {
      return maxVelocityError;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityReferenceAlpha()
   {
      return velocityReferenceAlpha;
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
         if (maxVelocityError != other.getMaxVelocityError())
            return false;
         if (velocityReferenceAlpha != other.getVelocityReferenceAlpha())
            return false;
         if (velocityResetMode != other.getVelocityResetMode())
            return false;
         if (disableAccelerationIntegration != other.getDisableAccelerationIntegration())
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
            + ", max position error: " + maxPositionError + ", maxVelocityError: " + maxVelocityError + ", velocityReferenceAlpha: " + velocityReferenceAlpha
            + ", velocityResetMode: " + velocityResetMode;
   }
}
