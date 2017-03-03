package us.ihmc.llaQuadruped;

import us.ihmc.euclid.tuple2D.Vector2D;

import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;

public class LLAQuadrupedPositionBasedCrawlControllerParameters implements QuadrupedPositionBasedCrawlControllerParameters
{
   private final Vector2D desiredCoMOffset = new Vector2D(0.02, 0.0);
   
   @Override
   public double getMinimumVelocityForFullSkew()
   {
      return 0.1;
   }

   @Override
   public double getMinimumDistanceFromSameSideFoot()
   {
      return 0.04;
   }

   @Override
   public double getStanceLength()
   {
      return 0.6;
   }

   @Override
   public double getStanceWidth()
   {
      return 0.3;
   }

   @Override
   public double getMaxForwardSkew()
   {
      return 0.2;
   }
   
   @Override
   public double getMaxLateralSkew()
   {
      return 0.05;
   }
   @Override
   public double getMaxYawPerStep()
   {
      return 0.25;
   }

   @Override
   public double getInitalCoMHeight()
   {
      return 0.35;
   }

   @Override
   public double getDefaultSwingHeight()
   {
      return 0.075;
   }

   @Override
   public double getDefaultSwingDuration()
   {
      return 1.0;
   }

   @Override
   public double getDefaultSubCircleRadius()
   {
      return 0.2;
   }

   @Override
   public double getDefaultCoMCloseToFinalDesiredTransitionRadius()
   {
      return 0.1;
   }

   @Override
   public Vector2D getDefaultDesiredCoMOffset()
   {
      return desiredCoMOffset;
   }

   @Override
   public double getMaxYawRate()
   {
      return 0.2;
   }

   @Override
   public double getXOffsetFromCenterOfHips()
   {
      return -0.04;
   }
}
