package us.ihmc.wholeBodyControlCore.pidGains;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.wholeBodyControlCore.pidGains.implementations.PID3DGains;

/**
 * Write and read interface for PID gains in three dimensions.
 */
public interface PID3DGainsBasics extends PID3DGainsReadOnly, Settable<PID3DGainsReadOnly>
{
   /**
    * Sets the proportional PID gains for all three dimensions.
    *
    * @param proportionalGainX the new proportional gain for the x direction.
    * @param proportionalGainY the new proportional gain for the y direction.
    * @param proportionalGainZ the new proportional gain for the z direction.
    */
   void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ);

   /**
    * Sets the derivative PID gains for all three dimensions.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio.
    * </p>
    * 
    * @param derivativeGainX the new derivative gain for the x direction.
    * @param derivativeGainY the new derivative gain for the y direction.
    * @param derivativeGainZ the new derivative gain for the z direction.
    */
   void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ);

   /**
    * Sets the damping ratio for the PID gains for all three dimensions.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio.
    * </p>
    *
    * @param dampingRatio the new damping ratio.
    */
   default void setDampingRatios(double dampingRatio)
   {
      setDampingRatios(dampingRatio, dampingRatio, dampingRatio);
   }

   /**
    * Sets the damping ratio for the PID gains for all three dimensions.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio.
    * </p>
    *
    * @param dampingRatioX the new damping ratio for the x direction.
    * @param dampingRatioY the new damping ratio for the y direction.
    * @param dampingRatioZ the new damping ratio for the z direction.
    */
   default void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      setDerivativeGains(GainCalculator.computeDerivativeGain(getProportionalGains()[0], dampingRatioX),
                         GainCalculator.computeDerivativeGain(getProportionalGains()[1], dampingRatioY),
                         GainCalculator.computeDerivativeGain(getProportionalGains()[2], dampingRatioZ));
   }

   /**
    * Sets the integral PID gains for all three dimensions and sets the maximum integral error allowed
    * by the PID controller.
    *
    * @param integralGainX    the new integral gain for the x direction.
    * @param integralGainY    the new integral gain for the y direction.
    * @param integralGainZ    the new integral gain for the z direction.
    * @param maxIntegralError the new maximum integral error.
    */
   void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError);

   /**
    * Sets the maximum feedback value and the maximum feedback rate (derivative of the feedback value)
    * allowed by the PID controller.
    *
    * @param maxFeedback     the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    */
   void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate);

   /**
    * Sets the maximum error in the derivative input to the controller that will be considered.
    *
    * @param maxDerivativeError the new maximum derivative error.
    */
   void setMaxDerivativeError(double maxDerivativeError);

   /**
    * Sets the maximum error in the proportional input to the controller that will be considered.
    *
    * @param maxProportionalError the new maximum proportional error.
    */
   void setMaxProportionalError(double maxProportionalError);

   /**
    * Sets the proportional PID gains for all three dimensions from a double array. The provided array
    * must be of length three.
    *
    * @param proportionalGains the new proportional gains for the three directions.
    * @see PID3DGainsBasics#setProportionalGains(double, double, double)
    */
   default void setProportionalGains(double[] proportionalGains)
   {
      PID3DGainsReadOnly.checkArrayLength(proportionalGains);
      setProportionalGains(proportionalGains[0], proportionalGains[1], proportionalGains[2]);
   }

   /**
    * Sets the proportional PID gains for all three dimensions from a 3D tuple.
    *
    * @param proportionalGains the new proportional gains for the three directions.
    * @see PID3DGainsBasics#setProportionalGains(double, double, double)
    */
   default void setProportionalGains(Tuple3DReadOnly proportionalGains)
   {
      setProportionalGains(proportionalGains.getX(), proportionalGains.getY(), proportionalGains.getZ());
   }

   /**
    * Sets the derivative PID gains for all three dimensions from a double array. The provided array
    * must be of length three.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link PID3DGains}.
    * </p>
    * 
    * @param derivativeGains the new derivative gains for the three directions.
    * @see PID3DGainsBasics#setDerivativeGains(double, double, double)
    */
   default void setDerivativeGains(double[] derivativeGains)
   {
      PID3DGainsReadOnly.checkArrayLength(derivativeGains);
      setDerivativeGains(derivativeGains[0], derivativeGains[1], derivativeGains[2]);
   }

   /**
    * Sets the derivative PID gains for all three dimensions from a 3D tuple.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link PID3DGains}.
    * </p>
    * 
    * @param derivativeGains the new derivative gains for the three directions.
    * @see PID3DGainsBasics#setDerivativeGains(double, double, double)
    */
   default void setDerivativeGains(Tuple3DReadOnly derivativeGains)
   {
      setDerivativeGains(derivativeGains.getX(), derivativeGains.getY(), derivativeGains.getZ());
   }

   /**
    * Sets the derivative PID gains for all three dimensions from a double array. The provided array
    * must be of length three. Also sets the maximum integral error allowed by the PID controller.
    *
    * @param integralGains    the new integral gains for the three directions.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGainsBasics#setIntegralGains(double, double, double, double)
    */
   default void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      PID3DGainsReadOnly.checkArrayLength(integralGains);
      setIntegralGains(integralGains[0], integralGains[1], integralGains[2], maxIntegralError);
   }

   /**
    * Sets the derivative PID gains for all three dimensions from a 3D tuple. Also sets the maximum
    * integral error allowed by the PID controller.
    *
    * @param integralGains    the new integral gains for the three directions.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGainsBasics#setIntegralGains(double, double, double, double)
    */
   default void setIntegralGains(Tuple3DReadOnly integralGains, double maxIntegralError)
   {
      setIntegralGains(integralGains.getX(), integralGains.getY(), integralGains.getZ(), maxIntegralError);
   }

   /**
    * Sets the proportional PID gains for all three dimensions to the provided gain.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @see PID3DGainsBasics#setProportionalGains(double, double, double)
    */
   default void setProportionalGains(double proportionalGain)
   {
      setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
   }

   /**
    * Sets the derivative PID gains for all three dimensions to the provided gain.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link PID3DGains}.
    * </p>
    * 
    * @param derivativeGain the new derivative gain for the three dimensions.
    * @see PID3DGainsBasics#setDerivativeGains(double, double, double)
    */
   default void setDerivativeGains(double derivativeGain)
   {
      setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }

   /**
    * Sets the integral PID gains for all three dimensions to the provided gain. Also sets the maximum
    * integral error allowed by the PID controller.
    *
    * @param integralGain     the new integral gain for the three dimensions.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGainsBasics#setIntegralGains(double, double, double, double)
    */
   default void setIntegralGains(double integralGain, double maxIntegralError)
   {
      setIntegralGains(integralGain, integralGain, integralGain, maxIntegralError);
   }

   /**
    * Sets the gains for all three directions to the provided values.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @param derivativeGain   the new derivative gain for the three dimensions.
    */
   default void setProportialAndDerivativeGains(double proportionalGain, double derivativeGain)
   {
      setProportionalGains(proportionalGain);
      setDerivativeGains(derivativeGain);
   }

   /**
    * Sets the gains for all three directions to the provided values.
    *
    * @param proportionalGain the new proportional gain for the three dimensions.
    * @param derivativeGain   the new derivative gain for the three dimensions.
    * @param integralGain     the new integral gain for the three dimensions.
    * @param maxIntegralError the new maximum integral error.
    */
   default void setGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setProportionalGains(proportionalGain);
      setDerivativeGains(derivativeGain);
      setIntegralGains(integralGain, maxIntegralError);
   }

   /**
    * Copies the gains and parameters from the provided {@link PID3DGainsReadOnly} parameters into
    * this.
    * <p>
    * Note, that depending on the implementation the gains might use a damping ratio to determine the
    * derivative gains from the current proportional gains and the current damping ratio. Those
    * implementations will typically update the damping ratio from the current proportional gain and
    * the provided derivative gain if the derivative gain is set. In that case the order in which the
    * proportional and derivative gains are set might influence the outcome. Typically, it is safe to
    * first set the proportional gain and then set the derivative gain or damping ratio. For an example
    * of this see {@link PID3DGains}.
    * </p>
    * 
    * @param other the new gains.
    */
   @Override
   default void set(PID3DGainsReadOnly other)
   {
      setProportionalGains(other.getProportionalGains());
      setDerivativeGains(other.getDerivativeGains());
      setIntegralGains(other.getIntegralGains(), other.getMaximumIntegralError());
      setMaxFeedbackAndFeedbackRate(other.getMaximumFeedback(), other.getMaximumFeedbackRate());
      setMaxDerivativeError(other.getMaximumDerivativeError());
      setMaxProportionalError(other.getMaximumProportionalError());
   }
}
