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
public class UpDownPolyonCheckPoints2D
{
   private final FramePoint2D centerPoint;
   private final ArrayList<FramePoint2D> points = new ArrayList<>();
   private final int numberOfVertices;
   private final double radius;

   public UpDownPolyonCheckPoints2D(int numberOfVertices, double radius)
   {
      this.numberOfVertices = numberOfVertices;
      this.radius = radius;

      centerPoint = new FramePoint2D();
      points.add(centerPoint); // center

      for (int i = 0; i < numberOfVertices; i++)
      {
         points.add(new FramePoint2D());
      }
   }

   public void reset(ReferenceFrame referenceFrame)
   {
      centerPoint.setToZero(referenceFrame);

      double angleStepSize = Math.PI * 2.0 / (double) numberOfVertices;

      for (int i = 0; i < numberOfVertices; i++)
      {
         double x = radius * Math.cos(i * angleStepSize);
         double y = radius * Math.sin(i * angleStepSize);
         points.get(i+ 1).setIncludingFrame(referenceFrame, x, y);
      }
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      centerPoint.changeFrame(referenceFrame);

      for (int i = 0; i < numberOfVertices; i++)
      {
         points.get(i+ 1).changeFrame(referenceFrame);
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

   public ReferenceFrame getReferenceFrame()
   {
      return centerPoint.getReferenceFrame();
   }
}
