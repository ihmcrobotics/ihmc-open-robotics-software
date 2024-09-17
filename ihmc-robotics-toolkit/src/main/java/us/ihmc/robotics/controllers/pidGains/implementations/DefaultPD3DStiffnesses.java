package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.PDSE3Stiffnesses;
import us.ihmc.robotics.controllers.pidGains.PDSE3StiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPD3DStiffnesses;

import java.util.Arrays;

/**
 * Provides a default implementation for PD stiffnesses in three dimensions.
 * <p>
 * If this object is created a {@link GainCoupling} can be specified. This stiffness coupling is used in
 * case these PD stiffnesss will be used to create {@link YoPD3DStiffnesses}. In that case is it used to
 * determine what YoVariables to create for tuning. Note, that regardless of the specified stiffness
 * coupling the getters and setters in this implementation are designed for three dimensions.
 * </p>
 */
public class DefaultPD3DStiffnesses implements PD3DStiffnesses, Settable<DefaultPD3DStiffnesses>
{
   private double[] proportionalStiffnesses = new double[3];
   private double[] derivativeStiffnesses = new double[3];

   private double maxDerivativeError = Double.POSITIVE_INFINITY;
   private double maxProportionalError = Double.POSITIVE_INFINITY;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   public DefaultPD3DStiffnesses()
   {
   }

   public DefaultPD3DStiffnesses(PD3DStiffnessesReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(DefaultPD3DStiffnesses other) { PD3DStiffnesses.super.set(other);}

   @Override
   public double[] getProportionalStiffnesses()
   {
      return proportionalStiffnesses;
   }

   @Override
   public double[] getDerivativeStiffnesses()
   {
      return derivativeStiffnesses;
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate;
   }

   @Override
   public void setProportionalStiffnesses(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalStiffnesses[0] = proportionalGainX;
      proportionalStiffnesses[1] = proportionalGainY;
      proportionalStiffnesses[2] = proportionalGainZ;
   }

   @Override
   public void setDerivativeStiffnesses(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeStiffnesses[0] = derivativeGainX;
      derivativeStiffnesses[1] = derivativeGainY;
      derivativeStiffnesses[2] = derivativeGainZ;
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      this.maxFeedback = maxFeedback;
      this.maxFeedbackRate = maxFeedbackRate;
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      this.maxDerivativeError = maxDerivativeError;
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      this.maxProportionalError = maxProportionalError;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof DefaultPD3DStiffnesses)
      {
         DefaultPD3DStiffnesses other = (DefaultPD3DStiffnesses) object;
         if (!PD3DStiffnesses.super.equals(other))
            return false;
         return true;
      }
      else if (object instanceof PD3DStiffnessesReadOnly)
      {
         return PD3DStiffnesses.super.equals((PD3DStiffnessesReadOnly) object);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": kp: " + Arrays.toString(proportionalStiffnesses) + ", kd: " + Arrays.toString(derivativeStiffnesses);
   }
}
