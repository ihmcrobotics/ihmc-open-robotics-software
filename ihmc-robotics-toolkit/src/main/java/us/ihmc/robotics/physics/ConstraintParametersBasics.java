package us.ihmc.robotics.physics;

public interface ConstraintParametersBasics extends ConstraintParametersReadOnly
{
   default void set(ConstraintParametersReadOnly other)
   {
      setCoefficientOfRestitution(other.getCoefficientOfRestitution());
      setConstraintForceMixing(other.getConstraintForceMixing());
      setErrorReductionParameter(other.getErrorReductionParameter());
   }

   void setCoefficientOfRestitution(double coefficientOfRestitution);

   void setConstraintForceMixing(double constraintForceMixing);

   void setErrorReductionParameter(double errorReductionParameter);
}
