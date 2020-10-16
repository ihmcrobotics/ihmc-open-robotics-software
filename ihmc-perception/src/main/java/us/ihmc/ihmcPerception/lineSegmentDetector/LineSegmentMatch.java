package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.bytedeco.opencv.opencv_core.Scalar4i;

public class LineSegmentMatch
{
   Scalar4i previousLineSegment, currentLineSegment;

   private double angleCost;
   private double lengthCost;
   private double midPointCost;

   LineSegmentMatch(Scalar4i previousLineSegment, Scalar4i currentLineSegment)
   {
      this.previousLineSegment = previousLineSegment;
      this.currentLineSegment = currentLineSegment;
   }

   LineSegmentMatch(Scalar4i previousLineSegment, Scalar4i currentLineSegment, double angleCost, double midPointCost, double lengthCost)
   {
      this.previousLineSegment = previousLineSegment;
      this.currentLineSegment = currentLineSegment;
      this.angleCost = angleCost;
      this.midPointCost = midPointCost;
      this.lengthCost = lengthCost;
   }
}
