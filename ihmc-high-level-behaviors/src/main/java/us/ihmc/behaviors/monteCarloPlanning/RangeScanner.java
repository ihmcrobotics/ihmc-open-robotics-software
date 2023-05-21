package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Vector4D32;

import java.util.ArrayList;

public class RangeScanner
{
   private final int numPoints = 100;
   private final int maxRange = 10;

   public Point2D getClosestIntersectionPoint(Point2D start_point, Point2D end_point, Point2D obstacle_center, float obstacle_size_x, float obstacle_size_y)
   {
      Point2D closest_point = new Point2D(10000000, 10000000);

      float obstacle_min_x = obstacle_center.getX32() - obstacle_size_x;
      float obstacle_max_x = obstacle_center.getX32() + obstacle_size_x;
      float obstacle_min_y = obstacle_center.getY32() - obstacle_size_y;
      float obstacle_max_y = obstacle_center.getY32() + obstacle_size_y;

      // build polygon of obstacle with vertices
      Point2D[] polygon = new Point2D[4];
      polygon[0] = new Point2D(obstacle_min_x, obstacle_min_y);
      polygon[1] = new Point2D(obstacle_min_x, obstacle_max_y);
      polygon[2] = new Point2D(obstacle_max_x, obstacle_max_y);
      polygon[3] = new Point2D(obstacle_max_x, obstacle_min_y);

      // Get the closest intersection point of line segment with the obstacle polygon
      Point2D intersection_point = getClosestIntersectionPoint(start_point, end_point, polygon);

      // If the intersection point is closer than the current closest point, update the closest point
      if (intersection_point.distance(start_point) < closest_point.distance(start_point))
      {
         closest_point = intersection_point;
      }

      // check if intersection with simulation grid walls is closer than the current closest point
      intersection_point = getWallIntersectionPoint(start_point,
                                                    end_point,
                                                    new Point2D[] {new Point2D(0, 0), new Point2D(0, 100), new Point2D(100, 100), new Point2D(100, 0)});

      if (intersection_point.distance(start_point) < closest_point.distance(start_point))
      {
         closest_point = intersection_point;
      }

      return closest_point;
   }

   public ArrayList<Point2D> scan(Point2D pos, ArrayList<Vector4D32> obstacles)
   {
      ArrayList<Point2D> points = new ArrayList<>();
      for (int i = 0; i < numPoints; i++)
      {
         float theta = i * 2 * (float) Math.PI / numPoints;
         Point2D point = getScanPoint(pos, theta, obstacles);
         points.add(point);
      }

      return points;
   }

   public Point2D getScanPoint(Point2D pos, float theta, ArrayList<Vector4D32> obstacles)
   {
      // Get the end point of the ray
      Point2D end_point = new Point2D(pos.getX32() + maxRange * (float) Math.cos(theta), pos.getY32() + maxRange * (float) Math.sin(theta));

      // Get the intersection point with the obstacles
      Point2D intersection_point = getIntersectionPoint(pos, end_point, obstacles);

      if (intersection_point.distance(pos) > maxRange)
      {
         intersection_point = end_point;
      }

      return intersection_point;
   }

   public Point2D getIntersectionPoint(Point2D start_point, Point2D end_point, ArrayList<Vector4D32> obstacles)
   {

      // set closest_point to max, max
      Point2D closest_point = new Point2D(10000000, 10000000);

      for (Vector4D32 obstacle : obstacles)
      {
         Point2D obstacle_center = new Point2D(obstacle.getX32(), obstacle.getY32());
         float obstacle_size_x = obstacle.getZ32();
         float obstacle_size_y = obstacle.getS32();

         // Get all four corners of the obstacle
         float obstacle_min_x = obstacle_center.getX32() - obstacle_size_x;
         float obstacle_max_x = obstacle_center.getX32() + obstacle_size_x;
         float obstacle_min_y = obstacle_center.getY32() - obstacle_size_y;
         float obstacle_max_y = obstacle_center.getY32() + obstacle_size_y;

         // build polygon of obstacle with vertices
         Point2D[] polygon = new Point2D[4];
         polygon[0] = new Point2D(obstacle_min_x, obstacle_min_y);
         polygon[1] = new Point2D(obstacle_min_x, obstacle_max_y);
         polygon[2] = new Point2D(obstacle_max_x, obstacle_max_y);
         polygon[3] = new Point2D(obstacle_max_x, obstacle_min_y);

         // Get the closest intersection point of line segment with the obstacle polygon
         Point2D intersection_point = getClosestIntersectionPoint(start_point, end_point, polygon);

         // If the intersection point is closer than the current closest point, update the closest point
         if (intersection_point.distance(start_point) < closest_point.distance(start_point))
         {
            closest_point = intersection_point;
         }

         // check if intersection with simulation grid walls is closer than the current closest point
         intersection_point = getWallIntersectionPoint(start_point,
                                                       end_point,
                                                       new Point2D[] {new Point2D(0, 0), new Point2D(0, 100), new Point2D(100, 100), new Point2D(100, 0)});

         if (intersection_point.distance(start_point) < closest_point.distance(start_point))
         {
            closest_point = intersection_point;
         }
      }

      return closest_point;
   }

   public Point2D getClosestIntersectionPoint(Point2D start_point, Point2D end_point, Point2D[] polygon)
   {

      Point2D closest_point = new Point2D(10000000, 10000000);

      // find (m,c) for start-end line segment
      float m1;
      float c1;
      if (end_point.getX32() - start_point.getX32() == 0)
      {
         m1 = 0;
      }
      else
      {
         m1 = (end_point.getY32() - start_point.getY32()) / (end_point.getX32() - start_point.getX32());
      }
      c1 = start_point.getY32() - m1 * start_point.getX32();

      for (int i = 0; i < polygon.length; i++)
      {
         Point2D p1 = polygon[i];
         Point2D p2 = polygon[(i + 1) % polygon.length];

         float x = 0;
         float y = 0;

         if (p1.getX32() == p2.getX32())
         {
            y = m1 * p1.getX32() + c1;
            x = p1.getX32();

            if (y < Math.min(p1.getY32(), p2.getY32()) || y > Math.max(p1.getY32(), p2.getY32()))
            {
               continue;
            }
         }
         else if (p1.getY32() == p2.getY32() && m1 != 0)
         {
            x = (p1.getY32() - c1) / m1;
            y = p1.getY32();

            if (x < Math.min(p1.getX32(), p2.getX32()) || x > Math.max(p1.getX32(), p2.getX32()))
            {
               continue;
            }
         }

         Point2D point = new Point2D(x, y);

         if (point.distance(start_point) < closest_point.distance(start_point))
         {
            closest_point = point;
         }
      }

      return closest_point;
   }

   public Point2D getWallIntersectionPoint(Point2D start_point, Point2D end_point, Point2D[] polygon)
   {

      Point2D good_point = new Point2D(10000000, 10000000);

      // find (m,c) for start-end line segment
      float m1;
      float c1;
      if (end_point.getX32() - start_point.getX32() == 0)
      {
         m1 = 0;
      }
      else
      {
         m1 = (end_point.getY32() - start_point.getY32()) / (end_point.getX32() - start_point.getX32());
      }
      c1 = start_point.getY32() - m1 * start_point.getX32();

      for (int i = 0; i < polygon.length; i++)
      {
         Point2D p1 = polygon[i];
         Point2D p2 = polygon[(i + 1) % polygon.length];

         float x = 0;
         float y = 0;

         if (p1.getX32() == p2.getX32())
         {
            y = m1 * p1.getX32() + c1;
            x = p1.getX32();

            if (y < Math.min(p1.getY32(), p2.getY32()) || y > Math.max(p1.getY32(), p2.getY32()))
            {
               continue;
            }
         }
         else if (p1.getY32() == p2.getY32() && m1 != 0)
         {
            x = (p1.getY32() - c1) / m1;
            y = p1.getY32();

            if (x < Math.min(p1.getX32(), p2.getX32()) || x > Math.max(p1.getX32(), p2.getX32()))
            {
               continue;
            }
         }

         Point2D point = new Point2D(x, y);

         float ratio = (point.getX32() - end_point.getX32()) / (start_point.getX32() - end_point.getX32());

         if (ratio < 0 || ratio > 1)
         {
            continue;
         }

         good_point = point;
      }

      return good_point;
   }

   public int getNumPoints()
   {
      return numPoints;
   }

   public int getMaxRange()
   {
      return maxRange;
   }
}