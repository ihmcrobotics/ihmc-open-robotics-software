package us.ihmc.robotics.geometry;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class InPlaceConvexHullCalculator2d
{
   public static boolean isConvexAndClockwise(List<? extends Point2DReadOnly> points)
   {
      int n = points.size();
      return isConvexAndClockwise(points, n);
   }

   /**
    * isConvex
    * Returns true if clockwiseOrderedListOfVertices forms a convex polygon,
    *
    * @param clockwiseOrderedListOfVertices ArrayList
    * @param n is the number of useful elements assuming they are positioned in [0; n-1]. The others won't be used. 
    * @return boolean
    */
   public static boolean isConvexAndClockwise(List<? extends Point2DReadOnly> points, int n)
   {
      if (n > points.size())
      {
         throw new RuntimeException("n must be less or equal to points.size()");
      }
      
      Point2DReadOnly temp;
      // get the y max (if == get x min) point
      Point2DReadOnly curr;
      Point2DReadOnly firstPoint = points.get(0);
      int firstIndex = 0;
      for (int i = 1; i < n; i++)
      {
         temp = points.get(i);
         if (temp.getX() <= firstPoint.getX())
         {
            if (temp.getX() != firstPoint.getX() || temp.getY() > firstPoint.getY())
            {
               firstPoint = temp;
               firstIndex = i;
            }
         }
      }

      // move the minimum point to the front
      double bestAngle, prevAngle = Math.PI;

      for (int i = firstIndex; i < n + firstIndex; i++)
      {
         curr = points.get(i % n);
         temp = points.get((i + 1) % n);

         // should be counter-clockwise most point
         bestAngle = getAngle(curr, temp);

         // if turn counter-clockwise from previous
         if (bestAngle > prevAngle)
         {
            return false;
         }

         prevAngle = bestAngle;
      }

      //all points in clockwise hull
      return true;
   }

   /** Use inPlaceGiftWrapConvexHull2d(List<Point2d> points, int originalSize) instead */
   @Deprecated
   public static List<Point2D> inPlaceGiftWrapConvexHull2d(List<Point2D> points)
   {
      int originalSize = points.size();
      int newSize = inPlaceGiftWrapConvexHull2d(points, originalSize);
      for (int i = originalSize - 1; i > newSize - 1; i--)
      {
         points.remove(i);
      }
      return points;
   }

   /** 
    * Do the same as the other inPlaceGiftWrapConvexHull2d() but doesn't remove any element from the List.
    * Instead, it puts the garbage elements at the end of the List and returns the number of elements defining the convex hull.
    * @param points: List<Point2d> that can contains garbage elements.
    * @param originalSize: int that refers the number of significant elements. They have to be in [0; originalSize-1].
    */
   public static int inPlaceGiftWrapConvexHull2d(List<? extends Point2DReadOnly> points, int originalSize)
   {
      int size = originalSize;

      // get the y max of the x min
      Point2DReadOnly temp;
      Point2DReadOnly firstPoint = points.get(0);
      int bestIndex = 0;
      for (int i = 1; i < size; i++)
      {
         temp = points.get(i);
         if (temp.getX() <= firstPoint.getX())
         {
            if (temp.getX() != firstPoint.getX() || temp.getY() > firstPoint.getY())
            {
               firstPoint = temp;
               bestIndex = i;
            }
         }
      }

      Collections.swap(points, bestIndex, 0);

      int j;
      Point2DReadOnly curr;
      double tempAngle, bestAngle;
      for (int i = 0; i < size; i++)
      {
         curr = points.get(i);

         // find the counter-clockwise most point
         bestIndex = 0;
         bestAngle = getAngle(curr, points.get(0));
         for (j = i + 1; j < size; j++)
         {
            temp = points.get(j);
            // remove duplicate points
            if (curr.epsilonEquals(temp, 1e-7))
            {
               Collections.swap(points, j, size-1);
               j--;
               size--;
               continue;
            }

            tempAngle = getAngle(curr, temp);

            if (tempAngle >= bestAngle && (tempAngle - bestAngle <= Math.PI))
            {
               bestAngle = tempAngle;
               bestIndex = j;
            }
         }

         // if got back to the first point: remove non hull points off the end
         if (bestIndex == 0)
         {
            return i+1;
         }

         //swap hull element up to next spot
         Collections.swap(points, i + 1, bestIndex);
      }

      //all points in hull
      return size;
   }

   public static double getAngle(Point2DReadOnly from, Point2DReadOnly to)
   {
      if (from.equals(to))
         return 0;

      double angle = Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());

      if (angle <= Math.PI / 2)
         angle += Math.PI / 2;
      else
         angle -= Math.PI * 3 / 2;

      return angle;
   }
}
