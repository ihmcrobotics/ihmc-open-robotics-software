package us.ihmc.robotics.hyperCubeTree;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphericalLinearResolutionProvider implements ResolutionProvider
{
   private static final boolean DEBUG = false;
   private final FramePoint center;
   private final FramePoint centerInWorldTemp;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double[] tempCenter = new double[3];
   private final OneDimensionalBounds linearRegion;
   private final double centerResolution;
   private final double exteriorResolution;
   private final double outerRadiusSquared;
   private final double innerRadiusSquared;
   private final LowPassTimingReporter timer = new LowPassTimingReporter(5);

   public SphericalLinearResolutionProvider(FramePoint center, double radius1, double innerResolution, double radius2, double outerResolution)
   {
      this(center, new OneDimensionalBounds(Math.abs(radius1), Math.abs(radius2)), innerResolution, outerResolution);
   }

   public SphericalLinearResolutionProvider(FramePoint center, OneDimensionalBounds bounds, double innerResolution, double exteriorResolution)
   {
      this.center = center;
      centerInWorldTemp = new FramePoint(center);
      centerInWorldTemp.changeFrame(worldFrame);
      this.linearRegion = bounds;
      this.centerResolution = innerResolution;
      this.exteriorResolution = exteriorResolution;
      outerRadiusSquared = linearRegion.max() * linearRegion.max();
      innerRadiusSquared = linearRegion.min() * linearRegion.min();
      if (DEBUG)
         timer.setupRecording("SphericalLinearResolutionProvider", "getResolution", 5000, 5000);
   }

   public double getResolution(double[] location)
   {
      timer.startTime();
      double distanceSquared = 0;


      centerInWorldTemp.setIncludingFrame(center);
      centerInWorldTemp.changeFrame(worldFrame);
      centerInWorldTemp.getPoint().get(tempCenter);

      for (int i = 0; i < location.length; i++)
      {
         double difference = location[i] - tempCenter[i];
         distanceSquared += difference * difference;
      }

      if (outerRadiusSquared <= (distanceSquared))
         return exteriorResolution;
      if (innerRadiusSquared >= distanceSquared)
         return centerResolution;
      double distance = Math.sqrt(distanceSquared);
      double resolution = centerResolution + (exteriorResolution - centerResolution) * linearRegion.unScale(distance);
      timer.endTime();

      return resolution;
   }

   public double getMinResolution()
   {
      return centerResolution;
   }


}
