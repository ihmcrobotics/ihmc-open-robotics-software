package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * <p>Title: </p>
 *
 * <p>Description: calculates the convex hull from an ArrayList of Point2d
 * Uses a marriage-before-conquer algorithm, as published by KirkPatrick and Seidel, 1983,
 * potentially with worst case time complexity O(n log(H)), where n is the size of the input set
 * and H is the size of the output set.
 * For now, a random method is used instead of the median method.
 * The linear time median methods must be implemented to make it O(n log(H)).
 *
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author Twan Koolen
 * @version 1.0
 */

//Semi-TODO: use median algorithms instead of random algorithms

/** Use InPlaceConvexHullCalculator2d instead */
@Deprecated
public class ConvexHullCalculator2d
{
   private static final Random random = new Random(100L);

   /**
    * getConvexHull
    * Returns the convex hull of any unordered ArrayList of Point2d's.
    * The returned points will be ordered clockwise, starting from the point
    * with the maximum y-coordinate value that is contained in the set of points
    * with the minimum x-coordinate value ('upper rightmost' vertex).
    *
    * References are kept intact, so the output is mutable.
    *
    * @param pointList ArrayList
    * @return ArrayList the convex hull
    */
   public static void getConvexHull(ArrayList<Point2d> convexHullToPack, ArrayList<Point2d> pointList)
   {   
      if (convexHullToPack == pointList) throw new RuntimeException("getConvexHull cannot be done in place!");
      
      // Find upper and lower hull:
      ArrayList<Point2d> upperHull = new ArrayList<Point2d>();
      getUpperHull(upperHull, pointList);
      ArrayList<Point2d> lowerHull = new ArrayList<Point2d>();
      getLowerHull(lowerHull, pointList);

      // Remove vertices on edges of upper and lower hull that have equal coordinates
      if (!upperHull.isEmpty() &&!lowerHull.isEmpty())
      {
         if ((lowerHull.get(0).x == upperHull.get(upperHull.size() - 1).x) && (lowerHull.get(0).y == upperHull.get(upperHull.size() - 1).y))
         {
            lowerHull.remove(0);
         }
      }

      if (!upperHull.isEmpty() &&!lowerHull.isEmpty())
      {
         if ((upperHull.get(0).x == lowerHull.get(lowerHull.size() - 1).x) && (upperHull.get(0).y == lowerHull.get(lowerHull.size() - 1).y))
         {
            lowerHull.remove(lowerHull.size() - 1);
         }
      }

      // Concatenate:
//      ArrayList<Point2d> result = new ArrayList<Point2d>(upperHull.size() + lowerHull.size());
      convexHullToPack.addAll(upperHull);
      convexHullToPack.addAll(lowerHull);

//    if (result.size() == 2)
//    {
//       throw new RuntimeException("pointList: " + pointList + "\n\nupperHull: " + upperHull + "\n\nlowerHull: " + lowerHull + "\n\nupperHull before removal: " + new ArrayList<Point2d>(getUpperHull(pointList)) + "\n\nlowerHull before removal:" + new ArrayList<Point2d>(getLowerHull(pointList)));
//    }

//      return result;
   }

   public static List<Point2d> getConvexHullCopy(List<Point2d> pointList)
   {
      // Copy:
      ArrayList<Point2d> copyList = new ArrayList<Point2d>(pointList.size());
      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d copy = new Point2d(pointList.get(i));
         copyList.add(copy);
      }

      ArrayList<Point2d> result = new ArrayList<Point2d>();
      getConvexHull(result, copyList);

      return result;

//    ArrayList<Point2d> ret = new ArrayList<Point2d>();
//
//    // Copy:
//    for (Point2d point : result)
//    {
//       ret.add(new Point2d(point));
//    }
//
//    return ret;
   }

   public static ArrayList<Point2d> getConvexHullCopy(double[][] pointListArray)
   {
      ArrayList<Point2d> pointList = new ArrayList<Point2d>();

      for (int i = 0; i < pointListArray.length; i++)
      {
         pointList.add(new Point2d(pointListArray[i]));
      }

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      getConvexHull(ret, pointList);

      return ret;
   }


   public static void getLowerHull(ArrayList<Point2d> lowerHullToPack, ArrayList<Point2d> pointList)
   {
      /*
       * Changes the points in pointList, but changes them back after the hull was found.
       * The reason for this is that the object reference remains the same.
       * Only the sign is changed, so there are no round-off errors.
       */

      lowerHullToPack.clear(); 
      
      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d point = pointList.get(i);
         point.x = -point.x;
         point.y = -point.y;
      }

      getUpperHull(lowerHullToPack, pointList);
      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d point = pointList.get(i);
         point.x = -point.x;
         point.y = -point.y;
      }
   }

   private static final ThreadLocal<Point2d[]> minMaxLocal = new ThreadLocal<Point2d[]>()
   {
      @Override
      protected Point2d[] initialValue()
      {
         Point2d[] minMax = new Point2d[2];
         return minMax;
      }

   };
   public static void getUpperHull(ArrayList<Point2d> upperHullToPack, ArrayList<Point2d> pointList)
   {
      if (upperHullToPack == pointList)
      {
         throw new RuntimeException("getUpperHull Doesn't work in place. Lists need to be different");
      }
      upperHullToPack.clear();
      
//    random = new Random(100L);

      // 1:
      Point2d[] minMax = minMaxLocal.get();
      findMinMax(minMax, pointList);
      Point2d min = minMax[0];
      Point2d max = minMax[1];

      if (min.equals(max))
      {
         upperHullToPack.add(min);
      }
      else
      {
         ArrayList<Point2d> T = new ArrayList<Point2d>();
         T.add(min);
         T.add(max);

         for(int i = 0; i <  pointList.size(); i++)
         {
            Point2d point = pointList.get(i);
            if ((point.x > min.x) && (point.x < max.x))
            {
               T.add(point);
            }
         }

         // 2:
//         result = connect(min, max, T);
         connect(upperHullToPack, min, max, T);
      }
   }

   /**
    * findMinMax Finds 'minimum' and 'maximum' points:
    * x(pMin) <= x(p(i)) <= x(pMax)
    * y(pMin >= y(p(i)) if x(pMin) = x(p(i))
    * y(pMax >= y(p(i)) if x(pMax) = x(p(i)) for i = 1,...,n
    *
    * @param pointList ArrayList
    * @return Point2d[] An array that has pMin as the first entry and pMax
    *   as the second entry
    */
   private static void findMinMax(Point2d[] minMax, ArrayList<Point2d> pointList)    // TESTED
   {
      Point2d min = null;
      Point2d max = null;

      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d point = pointList.get(i);
         if (min != null)
         {
            if ((point.x < min.x) || ((point.x == min.x) && (point.y >= min.y)))
            {
               min = point;
            }
         }
         else
         {
            min = point;
         }

         if (max != null)
         {
            if ((point.x > max.x) || ((point.x == max.x) && (point.y >= max.y)))
            {
               max = point;
            }
         }
         else
         {
            max = point;
         }
      }

      minMax[0] = min;
      minMax[1] = max;
   }

   private static void connect(ArrayList<Point2d> hullPointsToPack, Point2d min, Point2d max, ArrayList<Point2d> pointList)
   {
      hullPointsToPack.clear();
      connectRecursively(0, hullPointsToPack, min, max, pointList);
   }
   
   private static final MultiArrayListLocal pointListLeftLocal = new MultiArrayListLocal();
   
   private static final MultiArrayListLocal pointListRightLocal = new MultiArrayListLocal();
   
   private static void connectRecursively(int depth, ArrayList<Point2d> hullPointsToPack, Point2d min, Point2d max, ArrayList<Point2d> pointList)
   {
//      ArrayList<Point2d> hullPoints = new ArrayList<Point2d>();

      // 2.1:
      // implemented the random method for now. Linear time median algorithm would be faster. METHOD CRASHES IN RARE CASES, USED THE MEAN
//    ArrayList<Point2d> pointListWithoutMax = new ArrayList<Point2d>(pointList);
//    pointListWithoutMax.remove(max);
//    int randIndex = random.nextInt(pointListWithoutMax.size());
//    double a = pointListWithoutMax.get(randIndex).x;
      double a = (min.x + max.x) / 2.0;

      // 2.2:
      Point2d[] bridge = bridge(0, pointList, a);

      // 2.3:
      ArrayList<Point2d> pointListLeft = pointListLeftLocal.getAndClear(depth);
      pointListLeft.add(bridge[0]);

      ArrayList<Point2d> pointListRight = pointListRightLocal.getAndClear(depth);
      pointListRight.add(bridge[1]);

      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d point = pointList.get(i);
         if (point.x < bridge[0].x)
         {
            pointListLeft.add(point);
         }
         else if (point.x > bridge[1].x)
         {
            pointListRight.add(point);
         }
      }

      // 2.4:
      if (bridge[0].equals(min))
      {
         hullPointsToPack.add(bridge[0]);
      }
      else
      {
         connectRecursively(depth+1, hullPointsToPack, min, bridge[0], pointListLeft);
//         hullPoints.addAll(connect(min, bridge[0], pointListLeft));
      }

      if (bridge[1].equals(max))
      {
         hullPointsToPack.add(bridge[1]);
      }
      else
      {
         connectRecursively(depth+1, hullPointsToPack, bridge[1], max, pointListRight);
//         hullPoints.addAll(connect(bridge[1], max, pointListRight));
      }

//      return hullPoints;
   }

   
   private static MultiArrayListLocal candidatesLocal = new MultiArrayListLocal();
   private static ArrayOfPointListLocal pairsLocal = new ArrayOfPointListLocal();
   private static ThreadLocal<ArrayList<Double>> slopesLocal = new ThreadLocal<ArrayList<Double>>()
   {
      @Override
      protected ArrayList<Double> initialValue()
      {
         return new ArrayList<Double>(32);

      };

      @Override
      public ArrayList<Double> get()
      {
         ArrayList<Double> e = super.get();
         e.clear();
         return e;
      };
   };
   
   private static ArrayOfPointListLocal smallLocal = new ArrayOfPointListLocal();
   private static ArrayOfPointListLocal equalLocal = new ArrayOfPointListLocal();
   private static ArrayOfPointListLocal largeLocal = new ArrayOfPointListLocal();
   private static PointListLocal maxLocal = new PointListLocal();
   private static ArrayOfPointListLocal largeUnionEqualLocal = new ArrayOfPointListLocal();
   private static ArrayOfPointListLocal smallUnionEqualLocal = new ArrayOfPointListLocal();
   
   private static Point2d[] bridge(int depth, ArrayList<Point2d> pointList, double a)
   {
      Point2d[] bridge = new Point2d[2];

      // 0:

      // 1:
      if (pointList.size() == 2)
      {
         if (pointList.get(0).x < pointList.get(1).x)
         {
            bridge[0] = pointList.get(0);
            bridge[1] = pointList.get(1);
         }
         else
         {
            bridge[0] = pointList.get(1);
            bridge[1] = pointList.get(0);
         }

         return bridge;
      }


      // 2:     
      ArrayList<Point2d> candidates = candidatesLocal.getAndClear(depth);

      ArrayList<Point2d[]> pairs = pairsLocal.get();

      if (pointList.size() % 2 != 0)
      {
         candidates.add(pointList.get(pointList.size() - 1));
      }

      for (int i = 0; i < pointList.size() / 2; i++)
      {
         Point2d[] pair = new Point2d[2];

         if (pointList.get(2 * i).x <= pointList.get(2 * i + 1).x)
         {
            pair[0] = pointList.get(2 * i);
            pair[1] = pointList.get(2 * i + 1);
         }
         else
         {
            pair[0] = pointList.get(2 * i + 1);
            pair[1] = pointList.get(2 * i);
         }

         pairs.add(pair);
      }

      // 3:
      ArrayList<Double> slopes = slopesLocal.get();
      {
         int i = 0;    // This construction is used to cope with removing a pair while in the loop
         while (i < pairs.size())
         {
            Point2d[] pair = pairs.get(i);
            if (pair[0].x == pair[1].x)
            {
               if (pair[0].y > pair[1].y)
               {
                  candidates.add(pair[0]);
               }
               else
               {
                  candidates.add(pair[1]);
               }

               pairs.remove(i);
            }
            else
            {
               double slope = (pair[0].y - pair[1].y) / (pair[0].x - pair[1].x);
               slopes.add(slope);    // slopes is always 'synchronized' with pairs -> indices will match
               i++;
            }
         }
      }

      // My own addition:
      if (pairs.isEmpty())
      {
         return bridge(depth+1, candidates, a);
      }

      // 4:
      int randIndex = random.nextInt(slopes.size());
      double K = slopes.get(randIndex);

      // implemented the random method for now. Linear time median algorithm would be faster.
      // double K = computeMedian(slopes.toArray())

      // 5:
      ArrayList<Point2d[]> small = smallLocal.get();
      ArrayList<Point2d[]> equal = equalLocal.get();
      ArrayList<Point2d[]> large = largeLocal.get();

      for (int i = 0; i < pairs.size(); i++)
      {
         if (slopes.get(i) < K)
         {
            small.add(pairs.get(i));
         }
         else if (slopes.get(i) == K)
         {
            equal.add(pairs.get(i));
         }
         else
         {
            large.add(pairs.get(i));
         }
      }

      // 6:
      // Determine MAX:
      ArrayList<Point2d> max = maxLocal.get();
      double maxCriterionValue = Double.NEGATIVE_INFINITY;
      double epsilon = 1e-6;

      for(int i = 0; i <  pointList.size(); i++)
      {
         Point2d point = pointList.get(i);
         double criterionValue = point.y - K * point.x;
         if (criterionValue > maxCriterionValue + epsilon)    // If the criterion value is significantly larger
         {
            max.clear();
            max.add(point);
            maxCriterionValue = criterionValue;
         }
         else if (Math.abs(criterionValue - maxCriterionValue) <= epsilon)    // If the diff between criterionValue and maxCriterionvalue is zero-ish
         {
            max.add(point);
         }
      }

      // Find pK and pM:
      Point2d[] minMax = minMaxLocal.get(); 
      findMinMax(minMax, max);

      Point2d pK = minMax[0];
      Point2d pM = minMax[1];

      // 7:
      if ((pK.x <= a) && (pM.x > a))
      {
         bridge[0] = pK;
         bridge[1] = pM;

         return bridge;
      }

      // 8:
      if (pM.x <= a)
      {
         ArrayList<Point2d[]> largeUnionEqual = largeUnionEqualLocal.get();
         largeUnionEqual.addAll(large);
         largeUnionEqual.addAll(equal);

         for(int i = 0; i <  largeUnionEqual.size(); i++)
         {
            
            candidates.add(largeUnionEqual.get(i)[1]);
         }

         for(int i = 0; i <  small.size(); i++)
         {
            candidates.add(small.get(i)[0]);
            candidates.add(small.get(i)[1]);
         }
      }

      // 9:
      if (pK.x > a)
      {
         ArrayList<Point2d[]> smallUnionEqual = smallUnionEqualLocal.get();
         smallUnionEqual.addAll(small);
         smallUnionEqual.addAll(equal);

         for(int i = 0; i <  smallUnionEqual.size(); i++)
         {
            candidates.add(smallUnionEqual.get(i)[0]);
         }

         for(int i = 0; i <  large.size(); i++)
         {
            candidates.add(large.get(i)[0]);
            candidates.add(large.get(i)[1]);
         }
      }

      // 10:
      return (bridge(depth+1, candidates, a));
   }

   /**
    * isConvex
    * Returns true if clockwiseOrderedListOfVertices forms a convex polygon,
    * based on the number of sign changes encountered when 'walking along' the polygon edges
    *
    * @param clockwiseOrderedListOfVertices ArrayList
    * @return boolean
    */
   public static boolean isConvexAndClockwise(List<Point2d> pointList)
   {
      int n = pointList.size();
      return isConvexAndClockwise(pointList, n);
   }

   /**
    * isConvex
    * Returns true if clockwiseOrderedListOfVertices forms a convex polygon,
    * based on the number of sign changes encountered when 'walking along' the polygon edges
    *
    * @param clockwiseOrderedListOfVertices ArrayList
    * @param n is the number of useful elements assuming they are positioned in [0; n-1]. The others won't be used. 
    * @return boolean
    */
   public static boolean isConvexAndClockwise(List<Point2d> pointList, int n)
   {
      if (n < 3)
      {
         throw new RuntimeException("Method not applicable when less than 3 points are given");
      }
      else if(n > pointList.size())
      {
         throw new RuntimeException("n must be less or equal to pointList.size()");
      }

      // Create list of vertices with the last vertex equal to the first vertex:
      ArrayList<Point2d> closedListOfVertices = new ArrayList<Point2d>(pointList.subList(0, n));
      closedListOfVertices.add(new Point2d(pointList.get(0)));

      // Compute the differences between the vertices:
      ArrayList<Vector2d> differences = new ArrayList<Vector2d>();
      for (int i = 0; i < n; i++)
      {
         differences.add(new Vector2d(closedListOfVertices.get(i + 1)));
         differences.get(differences.size() - 1).sub(closedListOfVertices.get(i));
      }

      // 1. Check for sign changes:
      // Ensures that the polygon is convex
      int signChangesX = 0;
      int signChangesY = 0;

      for (int i = 0; i < (n - 1); i++)
      {
         if (sign(differences.get(i).x) != sign(differences.get(i + 1).x))
         {
            signChangesX++;
         }

         if (sign(differences.get(i).y) != sign(differences.get(i + 1).y))
         {
            signChangesY++;
         }
      }

      if (!((signChangesX <= 2) && (signChangesY <= 2)))
      {
         return false;
      }

      // 2. Check if the cross products are all positive:
      // Ensures that the points are ordered clockwise
      for (int i = 0; i < n; i++)
      {
         int j = (i + 1) % n;
         double cross = differences.get(i).x * differences.get(j).y - differences.get(j).x * differences.get(i).y;
         if (cross >= 0.0)
            return false;
      }

      return true;
   }

   private static int sign(double x)
   {
      return ((x < 0) ? -1 : 1);
   }
   
   private static class MultiArrayListLocal extends ThreadLocal<ArrayList<ArrayList<Point2d>>>
   {

      @Override
      protected ArrayList<ArrayList<Point2d>> initialValue()
      {
         ArrayList<ArrayList<Point2d>> result = new ArrayList<ArrayList<Point2d>>(32);
         for (int i=0; i<32; i++)
         {
            result.add(new ArrayList<Point2d>(32));
         }
         return result;
      }

      public ArrayList<Point2d> getAndClear(int depth)
      {
         ArrayList<ArrayList<Point2d>> multiListList = this.get();
         while (multiListList.size() <= depth)
         {
            multiListList.add(new ArrayList<Point2d>());
         }
         ArrayList<Point2d> multiList = multiListList.get(depth);
         multiList.clear(); 
         
         return multiList;
      }
   }

   private static class ArrayOfPointListLocal extends ThreadLocal<ArrayList<Point2d[]>>
   {
      @Override
      protected ArrayList<Point2d[]> initialValue()
      {
         ArrayList<Point2d[]> result = new ArrayList<Point2d[]>(16);
         return result;
      }

      public ArrayList<Point2d[]> get()
      {
         ArrayList<Point2d[]> e = super.get();
         e.clear();
         return e;
      }
   }
   
   private static class PointListLocal extends ThreadLocal<ArrayList<Point2d>>
   {

      @Override
      protected ArrayList<Point2d> initialValue()
      {
         ArrayList<Point2d> result = new ArrayList<Point2d>(16);
         return result;
      }

      public ArrayList<Point2d> get()
      {
         ArrayList<Point2d> e = super.get();
         e.clear();
         return e;
      }
   }
}
