package us.ihmc.quadrupedRobotics.footstepChooser;

public interface SwingTargetGeneratorParameters
{

   double getMinimumVelocityForFullSkew();

   double getMinimumDistanceFromSameSideFoot();

   double getStanceLength();

   double getStanceWidth();

   double getMaxSkew();

   double getMaxYawPerStep();

}