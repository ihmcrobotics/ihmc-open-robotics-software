package us.ihmc.robotics.controllers.pidGains;

/**
 * Interface for SE3 PID gains consisting of two PID gains in 3D. One for the
 * position control and one for the orientation control.
 */
public interface PIDSE3Gains extends PIDSE3GainsReadOnly
{
   /**
    * Returns the gains to be used for the position control.
    *
    * @return the position PID gains.
    */
   @Override
   public abstract PID3DGains getPositionGains();

   /**
    * Returns the gains to be used for the orientation control.
    *
    * @return the orientation PID gains.
    */
   @Override
   public abstract PID3DGains getOrientationGains();

   /**
    * Sets the position gains to the provided PID gains.
    *
    * @param positionGains the new gains for position control.
    */
   public default void setPositionGains(PID3DGainsReadOnly positionGains)
   {
      getPositionGains().set(positionGains);
   }

   /**
    * Sets the orientation gains to the provided PID gains.
    *
    * @param orientationGains the new gains for orientation control.
    */
   public default void setOrientationGains(PID3DGainsReadOnly orientationGains)
   {
      getOrientationGains().set(orientationGains);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param proportionalGainX the new proportional gain for the x direction.
    * @param proportionalGainY the new proportional gain for the y direction.
    * @param proportionalGainZ the new proportional gain for the z direction.
    * @see PID3DGains#setProportionalGains(double, double, double)
    */
   public default void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      getPositionGains().setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param derivativeGainX the new derivative gain for the x direction.
    * @param derivativeGainY the new derivative gain for the y direction.
    * @param derivativeGainZ the new derivative gain for the z direction.
    * @see PID3DGains#setDerivativeGains(double, double, double)
    */
   public default void setPositionDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      getPositionGains().setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param integralGainX the new integral gain for the x direction.
    * @param integralGainY the new integral gain for the y direction.
    * @param integralGainZ the new integral gain for the z direction.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGains#setIntegralGains(double, double, double)
    */
   public default void setPositionIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      getPositionGains().setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param proportionalGainX the new proportional gain for the x direction.
    * @param proportionalGainY the new proportional gain for the y direction.
    * @param proportionalGainZ the new proportional gain for the z direction.
    * @see PID3DGains#setProportionalGains(double, double, double)
    */
   public default void setOrientationProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      getOrientationGains().setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param derivativeGainX the new derivative gain for the x direction.
    * @param derivativeGainY the new derivative gain for the y direction.
    * @param derivativeGainZ the new derivative gain for the z direction.
    * @see PID3DGains#setDerivativeGains(double, double, double)
    */
   public default void setOrientationDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      getOrientationGains().setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param integralGainX the new integral gain for the x direction.
    * @param integralGainY the new integral gain for the y direction.
    * @param integralGainZ the new integral gain for the z direction.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGains#setIntegralGains(double, double, double)
    */
   public default void setOrientationIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      getOrientationGains().setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param proportionalGainX the new proportional gain for the three dimensions.
    * @see PID3DGains#setProportionalGains(double)
    */
   public default void setPositionProportionalGains(double proportionalGain)
   {
      getPositionGains().setProportionalGains(proportionalGain);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param derivativeGainX the new derivative gain for the three dimensions.
    * @see PID3DGains#setDerivativeGains(double)
    */
   public default void setPositionDerivativeGains(double derivativeGain)
   {
      getPositionGains().setDerivativeGains(derivativeGain);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param integralGain the new integral gain for the three dimensions.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGains#setIntegralGains(double, double)
    */
   public default void setPositionIntegralGains(double integralGain, double maxIntegralError)
   {
      getPositionGains().setIntegralGains(integralGain, maxIntegralError);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param proportionalGainX the new proportional gain for the three dimensions.
    * @see PID3DGains#setProportionalGains(double)
    */
   public default void setOrientationProportionalGains(double proportionalGain)
   {
      getOrientationGains().setProportionalGains(proportionalGain);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param derivativeGainX the new derivative gain for the three dimensions.
    * @see PID3DGains#setDerivativeGains(double)
    */
   public default void setOrientationDerivativeGains(double derivativeGain)
   {
      getOrientationGains().setDerivativeGains(derivativeGain);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param integralGainX the new integral gain for the three dimensions.
    * @param maxIntegralError the new maximum integral error.
    * @see PID3DGains#setIntegralGains(double, double)
    */
   public default void setOrientationIntegralGains(double integralGain, double maxIntegralError)
   {
      getOrientationGains().setIntegralGains(integralGain, maxIntegralError);
   }

   /**
    * Sets parameters in the position part of these SE3 gains.
    *
    * @param maxFeedback the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    * @see PID3DGains#setMaxFeedbackAndFeedbackRate(double, double)
    */
   public default void setPositionMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      getPositionGains().setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   /**
    * Sets parameters in the orientation part of these SE3 gains.
    *
    * @param maxFeedback the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    * @see PID3DGains#setMaxFeedbackAndFeedbackRate(double, double)
    */
   public default void setOrientationMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      getOrientationGains().setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   /**
    * Copies the gains and parameters from the provided {@link PIDSE3Gains}
    * parameters into this.
    *
    * @param other the new gains.
    */
   public default void set(PIDSE3GainsReadOnly other)
   {
      setOrientationGains(other.getOrientationGains());
      setPositionGains(other.getPositionGains());
   }
}