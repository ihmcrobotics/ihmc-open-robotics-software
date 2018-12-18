package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedFallDetector.FallDetectionType;

public interface QuadrupedFallDetectionParameters
{
   FallDetectionType getFallDetectionType();
   double getMaxPitch();
   double getMaxRoll();
   double getIcpDistanceOutsideSupportPolygon();
   double getMaxHeightError();

}
