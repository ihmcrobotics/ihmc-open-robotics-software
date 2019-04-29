package us.ihmc.humanoidBehaviors.upDownExploration;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.ArrayList;

/**
 * N corner points around one in the center in 2D, forming the points of a polygon plus a center one
 *
 * Like so:
 *
 * <pre>
 *       *
 *    *  *  *
 *      *  *
 * </pre>
 *
 * TODO add layers, like an onion
 */
public class PolygonPoints2D
{
   private final ArrayList<FramePoint2D> points = new ArrayList<>();
   private final FramePoint2D centerPoint;

   public PolygonPoints2D(int numberOfPoints, double radius, ReferenceFrame referenceFrame)
   {
      centerPoint = new FramePoint2D(referenceFrame);
      getPoints().add(centerPoint); // center

      double angleStepSize = Math.PI * 2.0 / (double) numberOfPoints;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = radius * Math.cos(i * angleStepSize);
         double y = radius * Math.sin(i * angleStepSize);
         getPoints().add(new FramePoint2D(referenceFrame, x, y));
      }
   }

   public void add(double x, double y)
   {
      for (FramePoint2D point : points)
      {
         point.add(x, y);
      }
   }

   public FramePoint2D getCenterPoint()
   {
      return centerPoint;
   }

   public ArrayList<FramePoint2D> getPoints()
   {
      return points;
   }
}
