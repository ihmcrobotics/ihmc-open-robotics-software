package us.ihmc.robotics.controllers.pidGains;

/**
 * Interface for SE3 PD apparent stiffnesses consisting of two PD stiffnesses in 3D. One for the
 * position control and one for the orientation control.
 */
public interface PDSE3Stiffnesses extends PDSE3StiffnessesReadOnly
{
   /**
    * Returns the gains to be used for the position control.
    *
    * @return the position PD stiffness.
    */
   @Override
   public abstract PD3DStiffnesses getPositionStiffnesses();

   /**
    * Returns the gains to be used for the orientation control.
    *
    * @return the orientation PD stiffness.
    */
   @Override
   public abstract PD3DStiffnesses getOrientationStiffnesses();

   /**
    * Sets the position gains to the provided PD stiffness.
    *
    * @param positionStiffnesses the new gains for position control.
    */
   public default void setPositionStiffnesses(PD3DStiffnessesReadOnly positionStiffnesses)
   {
      getPositionStiffnesses().set(positionStiffnesses);
   }

   /**
    * Sets the position gains to the provided PD stiffness.
    *
    * @param positionStiffnesses the new gains for position control.
    */
   public default void setPositionStiffnesses(PID3DGainsReadOnly positionStiffnesses)
   {
      getPositionStiffnesses().set(positionStiffnesses);
   }

   /**
    * Sets the orientation gains to the provided PD stiffness.
    *
    * @param orientationStiffnesses the new gains for orientation control.
    */
   public default void setOrientationStiffnesses(PD3DStiffnessesReadOnly orientationStiffnesses)
   {
      getOrientationStiffnesses().set(orientationStiffnesses);
   }

   /**
    * Sets the orientation gains to the provided PD stiffness.
    *
    * @param orientationStiffnesses the new gains for orientation control.
    */
   public default void setOrientationStiffnesses(PID3DGainsReadOnly orientationStiffnesses)
   {
      getOrientationStiffnesses().set(orientationStiffnesses);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param proportionalStiffnessX the new proportional gain for the x direction.
    * @param proportionalStiffnessY the new proportional gain for the y direction.
    * @param proportionalStiffnessZ the new proportional gain for the z direction.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double, double, double)
    */
   public default void setPositionProportionalStiffnesses(double proportionalStiffnessX, double proportionalStiffnessY, double proportionalStiffnessZ)
   {
      getPositionStiffnesses().setProportionalStiffnesses(proportionalStiffnessX, proportionalStiffnessY, proportionalStiffnessZ);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param derivativeStiffnessX the new derivative gain for the x direction.
    * @param derivativeStiffnessY the new derivative gain for the y direction.
    * @param derivativeStiffnessZ the new derivative gain for the z direction.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double, double, double)
    */
   public default void setPositionDerivativeStiffnesses(double derivativeStiffnessX, double derivativeStiffnessY, double derivativeStiffnessZ)
   {
      getPositionStiffnesses().setDerivativeStiffnesses(derivativeStiffnessX, derivativeStiffnessY, derivativeStiffnessZ);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param proportionalStiffnessX the new proportional gain for the x direction.
    * @param proportionalStiffnessY the new proportional gain for the y direction.
    * @param proportionalStiffnessZ the new proportional gain for the z direction.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double, double, double)
    */
   public default void setOrientationProportionalStiffnesses(double proportionalStiffnessX, double proportionalStiffnessY, double proportionalStiffnessZ)
   {
      getOrientationStiffnesses().setProportionalStiffnesses(proportionalStiffnessX, proportionalStiffnessY, proportionalStiffnessZ);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param derivativeStiffnessX the new derivative gain for the x direction.
    * @param derivativeStiffnessY the new derivative gain for the y direction.
    * @param derivativeStiffnessZ the new derivative gain for the z direction.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double, double, double)
    */
   public default void setOrientationDerivativeStiffnesses(double derivativeStiffnessX, double derivativeStiffnessY, double derivativeStiffnessZ)
   {
      getOrientationStiffnesses().setDerivativeStiffnesses(derivativeStiffnessX, derivativeStiffnessY, derivativeStiffnessZ);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param proportionalStiffness the new proportional gain for the three dimensions.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double)
    */
   public default void setPositionProportionalStiffnesses(double proportionalStiffness)
   {
      getPositionStiffnesses().setProportionalStiffnesses(proportionalStiffness);
   }

   /**
    * Sets gains in the position part of these SE3 gains.
    *
    * @param derivativeStiffness the new derivative gain for the three dimensions.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double)
    */
   public default void setPositionDerivativeStiffnesses(double derivativeStiffness)
   {
      getPositionStiffnesses().setDerivativeStiffnesses(derivativeStiffness);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param proportionalStiffness the new proportional gain for the three dimensions.
    * @see PD3DStiffnesses#setProportionalStiffnesses(double)
    */
   public default void setOrientationProportionalStiffnesses(double proportionalStiffness)
   {
      getOrientationStiffnesses().setProportionalStiffnesses(proportionalStiffness);
   }

   /**
    * Sets gains in the orientation part of these SE3 gains.
    *
    * @param derivativeStiffness the new derivative gain for the three dimensions.
    * @see PD3DStiffnesses#setDerivativeStiffnesses(double)
    */
   public default void setOrientationDerivativeStiffnesses(double derivativeStiffness)
   {
      getOrientationStiffnesses().setDerivativeStiffnesses(derivativeStiffness);
   }

   /**
    * Sets parameters in the position part of these SE3 gains.
    *
    * @param maxFeedback the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    * @see PD3DStiffnesses#setMaxFeedbackAndFeedbackRate(double, double)
    */
   public default void setPositionMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      getPositionStiffnesses().setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   /**
    * Sets parameters in the orientation part of these SE3 gains.
    *
    * @param maxFeedback the new maximum feedback.
    * @param maxFeedbackRate the new maximum feedback rate.
    * @see PD3DStiffnesses#setMaxFeedbackAndFeedbackRate(double, double)
    */
   public default void setOrientationMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      getOrientationStiffnesses().setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   /**
    * Copies the gains and parameters from the provided {@link PDSE3Stiffnesses}
    * parameters into this.
    *
    * @param other the new gains.
    */
   public default void set(PDSE3StiffnessesReadOnly other)
   {
      setOrientationStiffnesses(other.getOrientationStiffnesses());
      setPositionStiffnesses(other.getPositionStiffnesses());
   }

   /**
    * Copies the gains and parameters from the provided {@link PDSE3Stiffnesses}
    * parameters into this.
    *
    * @param other the new gains.
    */
   public default void set(PIDSE3GainsReadOnly other)
   {
      setOrientationStiffnesses(other.getOrientationGains());
      setPositionStiffnesses(other.getPositionGains());
   }
}