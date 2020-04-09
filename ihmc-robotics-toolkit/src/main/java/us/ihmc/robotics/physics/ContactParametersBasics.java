package us.ihmc.robotics.physics;

public interface ContactParametersBasics extends ContactParametersReadOnly, ConstraintParametersBasics
{
   default void set(ContactParametersReadOnly other)
   {
      ConstraintParametersBasics.super.set(other);
      setCoefficientOfFriction(other.getCoefficientOfFriction());
   }

   void setCoefficientOfFriction(double coefficientOfFriction);
}
