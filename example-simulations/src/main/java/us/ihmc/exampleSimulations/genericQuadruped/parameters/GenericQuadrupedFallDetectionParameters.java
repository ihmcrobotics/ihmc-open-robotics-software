package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedFallDetector.FallDetectionType;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;

public class GenericQuadrupedFallDetectionParameters implements QuadrupedFallDetectionParameters
{
   @Override
   public FallDetectionType getFallDetectionType()
   {
      return FallDetectionType.ALL;
   }

   @Override
   public double getMaxPitch()
   {
      return 0.5;
   }

   @Override
   public double getMaxRoll()
   {
      return 1.0;
   }

   @Override
   public double getIcpDistanceOutsideSupportPolygon()
   {
      return 0.15;
   }

   @Override
   public double getMaxHeightError()
   {
      return 0.15;
   }
}
