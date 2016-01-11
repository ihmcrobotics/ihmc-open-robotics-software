package us.ihmc.quadrupedRobotics.parameters;

import javax.vecmath.Vector2d;

public interface QuadrupedPositionBasedCrawlControllerParameters extends SwingTargetGeneratorParameters
{
   public abstract double getInitalCoMHeight();
   
   public abstract double getDefaultSwingHeight();

   public abstract double getDefaultSwingDuration();

   public abstract double getDefaultSubCircleRadius();

   public abstract double getMaxYawRate();

   public abstract double getDefaultCoMCloseToFinalDesiredTransitionRadius();
   
   public abstract Vector2d getDefaultDesiredCoMOffset();

}