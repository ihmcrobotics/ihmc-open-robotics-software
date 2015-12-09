package us.ihmc.quadrupedRobotics.parameters;

public class DefaultQuadrupedVirtualModelParameters implements QuadrupedVirtualModelParameters
{
   double defaultJointPositionLimitStiffness = 1000;
   double defaultJointPositionLimitDamping = 100;
   double defaultSoleCoefficientOfFriction = 0.8;

   @Override
   public double getDefaultJointPositionLimitStiffness()
   {
      return defaultJointPositionLimitStiffness;
   }

   @Override
   public double getDefaultJointPositionLimitDamping()
   {
      return defaultJointPositionLimitDamping;
   }

   @Override
   public double getDefaultSoleCoefficientOfFriction()
   {
      return defaultSoleCoefficientOfFriction;
   }

}
