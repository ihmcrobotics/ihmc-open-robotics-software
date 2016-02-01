package us.ihmc.quadrupedRobotics.parameters;

public interface SwingTargetGeneratorParameters
{

   double getMinimumVelocityForFullSkew();

   double getMinimumDistanceFromSameSideFoot();

   double getStanceLength();

   double getStanceWidth();

   double getMaxForwardSkew();
   
   double getMaxLateralSkew();

   double getMaxYawPerStep();

}