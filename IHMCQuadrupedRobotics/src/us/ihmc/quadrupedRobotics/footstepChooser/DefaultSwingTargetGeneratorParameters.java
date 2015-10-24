package us.ihmc.quadrupedRobotics.footstepChooser;

public class DefaultSwingTargetGeneratorParameters implements SwingTargetGeneratorParameters 
{
   private final double minimumVelocityForFullSkew = 0.1;
   private final double minimumDistanceFromSameSideFoot = 0.04;
   private final double strideLength = 0.34;
   private final double strideWidth = 0.24;
   private final double maxSkew = 0.29;
   private final double maxYaw = 0.25;
   
   @Override
   public double getMinimumVelocityForFullSkew()
   {
      return minimumVelocityForFullSkew;
   }

   @Override
   public double getMinimumDistanceFromSameSideFoot()
   {
      return minimumDistanceFromSameSideFoot;
   }

   @Override
   public double getStrideLength()
   {
      return strideLength;
   }

   @Override
   public double getStanceWidth()
   {
      return strideWidth;
   }

   @Override
   public double getMaxSkew()
   {
      return maxSkew;
   }

   @Override
   public double getMaxYawPerStep()
   {
      return maxYaw;
   }
}