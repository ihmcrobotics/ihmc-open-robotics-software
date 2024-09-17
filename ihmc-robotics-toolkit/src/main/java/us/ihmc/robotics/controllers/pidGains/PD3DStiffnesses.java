package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPD3DStiffnesses;

/**
 * Write and read interface for PD stiffness in three dimensions.
 */
public interface PD3DStiffnesses extends PD3DStiffnessesReadOnly
{
   /**
    * Sets the proportional PD stiffness for all three dimensions.
    *
    * @param proportionalGainX the new proportional gain for the x direction.
    * @param proportionalGainY the new proportional gain for the y direction.
    * @param proportionalGainZ the new proportional gain for the z direction.
    */
   public abstract void setProportionalStiffnesses(double proportionalGainX, double proportionalGainY, double proportionalGainZ);

   /**
    * Sets the derivative PD stiffness for all three dimensions.
    * <p>
    * Note, that depending on the implementation the stiffness might use a damping ratio to determine the
    * derivative stiffness from the current proportional stiffness and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative stiffness are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link DefaultPD3DStiffnesses}.
    * </p>
    * 
    * @param derivativeGainX the new derivative gain for the x direction.
    * @param derivativeGainY the new derivative gain for the y direction.
    * @param derivativeGainZ the new derivative gain for the z direction.
    */
   public abstract void setDerivativeStiffnesses(double derivativeGainX, double derivativeGainY, double derivativeGainZ);

   /**
    * Sets the maximum feedback value and the maximum feedback rate (derivative of the feedback value)
    * allowed by the PD controller.
    *
    * @param maxFeedback     the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    */
   public abstract void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate);

   /**
    * Sets the maximum error in the derivative input to the controller that will be considered.
    *
    * @param maxDerivativeError the new maximum derivative error.
    */
   public abstract void setMaxDerivativeError(double maxDerivativeError);

   /**
    * Sets the maximum error in the proportional input to the controller that will be considered.
    *
    * @param maxProportionalError the new maximum proportional error.
    */
   public abstract void setMaxProportionalError(double maxProportionalError);

   /**
    * Sets the proportional PD stiffness for all three dimensions from a double array. The provided array
    * must be of length three.
    *
    * @param proportionalStiffnesses the new proportional stiffness for the three directions.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double, double, double)
    */
   public default void setProportionalStiffnesses(double[] proportionalStiffnesses)
   {
      PD3DStiffnessesReadOnly.checkArrayLength(proportionalStiffnesses);
      setProportionalStiffnesses(proportionalStiffnesses[0], proportionalStiffnesses[1], proportionalStiffnesses[2]);
   }

   /**
    * Sets the proportional PD stiffness for all three dimensions from a 3D tuple.
    *
    * @param proportionalStiffnesses the new proportional stiffness for the three directions.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double, double, double)
    */
   public default void setProportionalStiffnesses(Tuple3DReadOnly proportionalStiffnesses)
   {
      setProportionalStiffnesses(proportionalStiffnesses.getX(), proportionalStiffnesses.getY(), proportionalStiffnesses.getZ());
   }

   /**
    * Sets the derivative PD stiffness for all three dimensions from a double array. The provided array
    * must be of length three.
    * <p>
    * Note, that depending on the implementation the stiffness might use a damping ratio to determine the
    * derivative stiffness from the current proportional stiffness and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative stiffness are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link DefaultPD3DStiffnesses}.
    * </p>
    * 
    * @param derivativeStiffnesses the new derivative stiffness for the three directions.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double, double, double)
    */
   public default void setDerivativeStiffnesses(double[] derivativeStiffnesses)
   {
      PD3DStiffnessesReadOnly.checkArrayLength(derivativeStiffnesses);
      setDerivativeStiffnesses(derivativeStiffnesses[0], derivativeStiffnesses[1], derivativeStiffnesses[2]);
   }

   /**
    * Sets the derivative PD stiffness for all three dimensions from a 3D tuple.
    * <p>
    * Note, that depending on the implementation the stiffness might use a damping ratio to determine the
    * derivative stiffness from the current proportional stiffness and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative stiffness are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link DefaultPD3DStiffnesses}.
    * </p>
    * 
    * @param derivativeStiffnesses the new derivative stiffness for the three directions.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double, double, double)
    */
   public default void setDerivativeStiffnesses(Tuple3DReadOnly derivativeStiffnesses)
   {
      setDerivativeStiffnesses(derivativeStiffnesses.getX(), derivativeStiffnesses.getY(), derivativeStiffnesses.getZ());
   }

   /**
    * Sets the proportional PD stiffness for all three dimensions to the provided gain.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double, double, double)
    */
   public default void setProportionalStiffnesses(double proportionalGain)
   {
      setProportionalStiffnesses(proportionalGain, proportionalGain, proportionalGain);
   }

   /**
    * Sets the derivative PD stiffness for all three dimensions to the provided gain.
    * <p>
    * Note, that depending on the implementation the stiffness might use a damping ratio to determine the
    * derivative stiffness from the current proportional stiffness and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative stiffness are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link DefaultPD3DStiffnesses}.
    * </p>
    * 
    * @param derivativeGain the new derivative gain for the three dimensions.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double, double, double)
    */
   public default void setDerivativeStiffnesses(double derivativeGain)
   {
      setDerivativeStiffnesses(derivativeGain, derivativeGain, derivativeGain);
   }

   /**
    * Sets the stiffness for all three directions to the provided values.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @param derivativeGain   the new derivative gain for the three dimensions.
    */
   public default void setProportialAndDerivativeStiffnesses(double proportionalGain, double derivativeGain)
   {
      setProportionalStiffnesses(proportionalGain);
      setDerivativeStiffnesses(derivativeGain);
   }

   /**
    * Sets the stiffness for all three directions to the provided values.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @param derivativeGain   the new derivative gain for the three dimensions.
    * @param integralGain     the new integral gain for the three dimensions.
    * @param maxIntegralError the new maximum integral error.
    */
   public default void setStiffnesses(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setProportionalStiffnesses(proportionalGain);
      setDerivativeStiffnesses(derivativeGain);
   }

   /**
    * Copies the stiffness and parameters from the provided {@link PD3DStiffnessesReadOnly} parameters into
    * this.
    * <p>
    * Note, that depending on the implementation the stiffness might use a damping ratio to determine the
    * derivative stiffness from the current proportional stiffness and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative stiffness are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link DefaultPD3DStiffnesses}.
    * </p>
    * 
    * @param other the new stiffness.
    */
   public default void set(PD3DStiffnessesReadOnly other)
   {
      setProportionalStiffnesses(other.getProportionalStiffnesses());
      setDerivativeStiffnesses(other.getDerivativeStiffnesses());
      setMaxFeedbackAndFeedbackRate(other.getMaximumFeedback(), other.getMaximumFeedbackRate());
      setMaxDerivativeError(other.getMaximumDerivativeError());
      setMaxProportionalError(other.getMaximumProportionalError());
   }
}
