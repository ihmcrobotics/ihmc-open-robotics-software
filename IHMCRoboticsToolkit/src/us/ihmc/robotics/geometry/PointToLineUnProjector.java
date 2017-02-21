package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.MathTools;

public class PointToLineUnProjector
{
   Point2D pointA = new Point2D();
   Vector2D difference = new Vector2D();
   double zA;
   double zDifference;
   boolean useX = true;
   double qA;
   double qMult;

   public void setLine(Point2D point0, Point2D point1, double point0z, double point1z)
   {
      pointA.set(point0);
      difference.sub(point1, point0);
      zA = point0z;
      zDifference = point1z - point0z;
      useX = Math.abs(difference.getX()) > Math.abs(difference.getY());

      if (useX)
      {
         qA = pointA.getX();
         qMult = 1 / difference.getX();
      }
      else
      {
         qA = pointA.getY();
         qMult = 1 / difference.getY();
      }

      if (!MathTools.isFinite(qMult))
         qMult = 0;

   }

   public double unProject(double x, double y)
   {
      double s = ((useX ? x : y) - qA) * qMult;
      return zA + zDifference * s;
   }
}
