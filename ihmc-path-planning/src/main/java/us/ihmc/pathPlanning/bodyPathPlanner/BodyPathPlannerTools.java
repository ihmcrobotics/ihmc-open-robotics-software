package us.ihmc.pathPlanning.bodyPathPlanner;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class BodyPathPlannerTools
{
   public static double calculateHeading(Point2DReadOnly startPosition, Point2DReadOnly endPosition)
   {
      double deltaX = endPosition.getX() - startPosition.getX();
      double deltaY = endPosition.getY() - startPosition.getY();

      return calculateHeading(deltaX, deltaY);
   }

   public static double calculateHeading(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double deltaX = endPosition.getX() - startPosition.getX();
      double deltaY = endPosition.getY() - startPosition.getY();

      return calculateHeading(deltaX, deltaY);
   }

   public static double calculateHeading(Vector2DReadOnly direction)
   {
      return calculateHeading(direction.getX(), direction.getY());
   }

   public static double calculateHeading(double deltaX, double deltaY)
   {
      return Math.atan2(deltaY, deltaX);
   }
}
