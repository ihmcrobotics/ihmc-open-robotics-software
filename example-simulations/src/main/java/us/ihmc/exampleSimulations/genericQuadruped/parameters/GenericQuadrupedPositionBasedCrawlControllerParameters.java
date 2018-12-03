package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPositionBasedCrawlControllerParameters;

public class GenericQuadrupedPositionBasedCrawlControllerParameters implements QuadrupedPositionBasedCrawlControllerParameters
{
   private final double minimumVelocityForFullSkew = 0.14;//0.1;
   private final double minimumDistanceFromSameSideFoot = 0.04;
   private final double strideLength = 1.05;//1.13;
   private final double strideWidth = 0.4;
   private final double maxForwardSkew = 0.26;
   private final double maxLateralSkew = 0.05;
   private final double maxYawPerStep = 0.1;
   private final double maxYawRate = 0.13;
   private final double initalCoMHeight = 0.55;//0.6;
   private final double swingHeight = 0.075;
   private final double swingDuration = 0.6;//1.25;
   private final double subCircleRadius = 0.09;

   private final double comCloseToFinalDesiredTransitionRadius = 0.10;
   private final Vector2D desiredCoMOffset = new Vector2D(0.02, 0.0);

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
   public double getStanceLength()
   {
      return strideLength;
   }

   @Override
   public double getStanceWidth()
   {
      return strideWidth;
   }

   @Override
   public double getMaxForwardSkew()
   {
      return maxForwardSkew;
   }
   
   @Override
   public double getMaxLateralSkew()
   {
      return maxLateralSkew;
   }

   @Override
   public double getMaxYawPerStep()
   {
      return maxYawPerStep;
   }

   @Override
   public double getInitialCoMHeight()
   {
      return initalCoMHeight;
   }

   @Override
   public double getDefaultSwingHeight()
   {
      return swingHeight;
   }

   @Override
   public double getDefaultSwingDuration()
   {
      return swingDuration;
   }

   @Override
   public double getDefaultSubCircleRadius()
   {
      return subCircleRadius;
   }

   @Override
   public double getDefaultCoMCloseToFinalDesiredTransitionRadius()
   {
      return comCloseToFinalDesiredTransitionRadius;
   }

   @Override
   public Vector2D getDefaultDesiredCoMOffset()
   {
      return desiredCoMOffset;
   }

   @Override
   public double getMaxYawRate()
   {
      return maxYawRate;
   }

   @Override
   public double getXOffsetFromCenterOfHips()
   {
      return -0.04;
   }
}
