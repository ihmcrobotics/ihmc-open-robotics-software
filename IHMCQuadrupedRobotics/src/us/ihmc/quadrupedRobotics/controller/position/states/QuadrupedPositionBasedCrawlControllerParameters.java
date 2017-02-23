package us.ihmc.quadrupedRobotics.controller.position.states;

import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.SwingTargetGeneratorParameters;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface QuadrupedPositionBasedCrawlControllerParameters extends SwingTargetGeneratorParameters
{
   public abstract double getInitalCoMHeight();
   
   public abstract double getDefaultSwingHeight();

   public abstract double getDefaultSwingDuration();

   public abstract double getDefaultSubCircleRadius();

   public abstract double getMaxYawRate();

   public abstract double getDefaultCoMCloseToFinalDesiredTransitionRadius();
   
   public abstract Vector2D getDefaultDesiredCoMOffset();

}