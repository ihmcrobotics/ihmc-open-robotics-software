package us.ihmc.humanoidBehaviors.upDownExploration;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.ArrayList;
import java.util.List;

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

   public PolygonPoints2D(int numberOfVertices, double radius, ReferenceFrame referenceFrame)
   {
      centerPoint = new FramePoint2D(referenceFrame);
      points.add(centerPoint); // center

      double angleStepSize = Math.PI * 2.0 / (double) numberOfVertices;

      for (int i = 0; i < numberOfVertices; i++)
      {
         double x = radius * Math.cos(i * angleStepSize);
         double y = radius * Math.sin(i * angleStepSize);
         points.add(new FramePoint2D(referenceFrame, x, y));
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

   public List<FramePoint2D> getPoints()
   {
      return points;
   }
}
