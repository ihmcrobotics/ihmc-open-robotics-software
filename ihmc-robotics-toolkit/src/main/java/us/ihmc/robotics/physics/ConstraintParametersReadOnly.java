package us.ihmc.robotics.physics;

public interface ConstraintParametersReadOnly
{
   double getCoefficientOfRestitution();

   double getConstraintForceMixing();

   double getErrorReductionParameter();
}
