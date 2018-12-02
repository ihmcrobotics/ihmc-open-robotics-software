package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.SwingTargetGeneratorParameters;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface QuadrupedPositionBasedCrawlControllerParameters extends SwingTargetGeneratorParameters
{
   double getInitialCoMHeight();
   
   double getDefaultSwingHeight();

   double getDefaultSwingDuration();

   double getDefaultSubCircleRadius();

   double getMaxYawRate();

   double getDefaultCoMCloseToFinalDesiredTransitionRadius();
   
   Vector2D getDefaultDesiredCoMOffset();

}