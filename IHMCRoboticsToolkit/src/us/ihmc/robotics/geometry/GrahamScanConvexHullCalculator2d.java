package us.ihmc.robotics.geometry;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;

import javax.vecmath.Point2d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;

/**
 * Adapted from http://algs4.cs.princeton.edu/99hull/GrahamScan.java.html
 *
 * TODO: make it return the points in clockwise order instead of counterclockwise
 * TODO: reduce garbage generation
 */
public class GrahamScanConvexHullCalculator2d
{
   private final Deque<Point2d> hull = new ArrayDeque<Point2d>();
   private final PolarOrderComparator polarOrderComparatorComparator = new PolarOrderComparator();
   private final YThenXComparator yThenXComparator = new YThenXComparator();

   public static GrahamScanConvexHullCalculator2d createFromPointList(double[][] pointList)
   {
      int numberOfPoints = pointList.length;
      ArrayList<Point2d> points = new ArrayList<Point2d>(numberOfPoints);
      for (int i = 0; i < numberOfPoints; i++)
      {
         points.add(new Point2d(pointList[i]));
      }

      return new GrahamScanConvexHullCalculator2d(points);
   }
   

   
   public GrahamScanConvexHullCalculator2d(List<Point2d> pointList)
   {
      // defensive copy
      int N = pointList.size();
      Point2d[] points = new Point2d[N];
      for (int i = 0; i < N; i++)
      {
         points[i] = pointList.get(i);
      }

      // preprocess so that points[0] has lowest y-coordinate; break ties by x-coordinate
      // points[0] is an extreme point of the convex hull
      // (alternatively, could do easily in linear time)
      Arrays.sort(points, yThenXComparator);

      // sort by polar angle with respect to base point points[0],
      // breaking ties by distance to points[0]
      polarOrderComparatorComparator.set(points[0]);
      Arrays.sort(points, 1, N, polarOrderComparatorComparator);

      hull.push(points[0]);    // p[0] is first extreme point

      // find index k1 of first point not equal to points[0]
      int k1;
      for (k1 = 1; k1 < N; k1++)
      {
         if (!equals(points[0], (points[k1])))
            break;
      }

      if (k1 == N)
         return;    // all points equal

      // find index k2 of first point not collinear with points[0] and points[k1]
      int k2;
      for (k2 = k1 + 1; k2 < N; k2++)
      {
         if (ccw(points[0], points[k1], points[k2]) != 0)
            break;
      }

      hull.push(points[k2 - 1]);    // points[k2-1] is second extreme point

      // Graham scan; note that points[N-1] is extreme point different from points[0]
      for (int i = k2; i < N; i++)
      {
         Point2d top = hull.pop();
         while (ccw(hull.peek(), top, points[i]) <= 0)
         {
            top = hull.pop();
         }

         hull.push(top);
         hull.push(points[i]);
      }

      assert isConvex();
   }

   // return extreme points on convex hull in counterclockwise order as an Iterable
   public Iterable<Point2d> hull()
   {
      Deque<Point2d> s = new ArrayDeque<Point2d>();
      for (Point2d p : hull)
      {
         s.push(p);
      }

      return s;
   }

   // check that boundary of hull is strictly convex
   private boolean isConvex()
   {
      int N = hull.size();
      if (N <= 2)
         return true;

      Point2d[] points = new Point2d[N];
      int n = 0;
      for (Point2d p : hull())
      {
         points[n++] = p;
      }

      for (int i = 0; i < N; i++)
      {
         if (ccw(points[i], points[(i + 1) % N], points[(i + 2) % N]) <= 0)
         {
            return false;
         }
      }

      return true;
   }


   public static boolean equals(Point2d p1, Point2d p2)
   {
      if (p2 == p1)
         return true;
      if (p2 == null)
         return false;
      if (p2.getClass() != p1.getClass())
         return false;

      return (p1.x == p2.x) && (p1.y == p2.y);
   }

   // is a->b->c a counter-clockwise turn?
   // -1 if clockwise, +1 if counter-clockwise, 0 if collinear
   public static int ccw(Point2d a, Point2d b, Point2d c)
   {
      double area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
      if (area2 < 0)
         return -1;
      else if (area2 > 0)
         return +1;
      else
         return 0;
   }


   // compare other points relative to polar angle (between 0 and 2pi) they make with this Point
   private class PolarOrderComparator implements Comparator<Point2d>
   {
      private Point2d point;

      public int compare(Point2d q1, Point2d q2)
      {
         double dx1 = q1.x - point.x;
         double dy1 = q1.y - point.y;
         double dx2 = q2.x - point.x;
         double dy2 = q2.y - point.y;

         if ((dy1 >= 0) && (dy2 < 0))
            return -1;    // q1 above; q2 below
         else if ((dy2 >= 0) && (dy1 < 0))
            return +1;    // q1 below; q2 above
         else if ((dy1 == 0) && (dy2 == 0))
         {    // 3-collinear and horizontal
            if ((dx1 >= 0) && (dx2 < 0))
               return -1;
            else if ((dx2 >= 0) && (dx1 < 0))
               return +1;
            else
               return 0;
         }
         else
            return -ccw(point, q1, q2);    // both above or below

         // Note: ccw() recomputes dx1, dy1, dx2, and dy2
      }

      public void set(Point2d point)
      {
         this.point = point;
      }
   }

   private static class YThenXComparator implements Comparator<Point2d>
   {
      public int compare(Point2d o1, Point2d o2)
      {
         if (o1.y < o2.y)
            return -1;
         if (o1.y > o2.y)
            return +1;
         if (o1.x < o2.x)
            return -1;
         if (o1.x > o2.x)
            return +1;

         return 0;
      }
   }

   // test client
   public static void main(String[] args)
   {
      SummaryStatistics statistics = new SummaryStatistics();

      Random random = new Random(251253L);
      int size = 8;
      ArrayList<Point2d> points = new ArrayList<Point2d>(size);

      int nTests = 1000000;
      for (int testNumber = 0; testNumber < nTests; testNumber++)
      {
         for (int i = 0; i < size; i++)
         {
            double x = random.nextDouble();
            double y = random.nextDouble();
            points.add(new Point2d(x, y));
         }

         long startTime = System.nanoTime();
         GrahamScanConvexHullCalculator2d graham = new GrahamScanConvexHullCalculator2d(points);
         graham.hull();
         long stopTime = System.nanoTime();

         long difference = stopTime - startTime;
         double millis = TimeUnit.MILLISECONDS.convert(difference, TimeUnit.NANOSECONDS);

         if (!graham.isConvex())
            throw new RuntimeException("Not convex!");

         if (testNumber > 1000)
            statistics.addValue(millis);
      }

      double mean = statistics.getMean();
      System.out.println("mean time millis = " + mean);
   }

   public List<Point2d> getClockwiseOrderedListOfPoints()
   {
      LinkedList<Point2d> ret = new LinkedList<Point2d>();

      for (Point2d point : hull())
      {
         ret.addLast(point);
      }
      
      return ret;
   }

  
}
