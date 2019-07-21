package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;

/** Class for merging two convex hulls. 
 * 
 *
 */
public class ConcaveHullMerger
{
   //TODO: Check with Sylvain to see if this already exists somewhere else. If not, clean it up nicely and move it somewhere.
   //TODO: Clockwise or counterclockwise order?

   public static PlanarRegion mergePlanarRegions(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      RigidBodyTransform transformTwoToWorld = new RigidBodyTransform();
      regionTwo.getTransformToWorld(transformTwoToWorld);

      RigidBodyTransform transformWorldToOne = new RigidBodyTransform();
      regionOne.getTransformToLocal(transformWorldToOne);

      RigidBodyTransform transformTwoToOne = new RigidBodyTransform(transformTwoToWorld);
      transformTwoToOne.multiply(transformWorldToOne);

      Point2D[] concaveHullTwoVertices = regionTwo.getConcaveHull();
      Point2D[] concaveHullTwoVerticesTransformed = new Point2D[concaveHullTwoVertices.length];

      for (int i = 0; i < concaveHullTwoVertices.length; i++)
      {
         Point3D point3D = new Point3D(concaveHullTwoVertices[i]);
         transformTwoToOne.transform(point3D);
         concaveHullTwoVerticesTransformed[i] = new Point2D(point3D);
      }

      ArrayList<Point2D> mergedConcaveHull = mergeConcaveHulls(regionOne.getConcaveHull(), concaveHullTwoVerticesTransformed);
      double depthThreshold = 0.001;
      ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<ConvexPolygon2D>();

      ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, depthThreshold, newPolygonsFromConcaveHull);

      RigidBodyTransform transformOneToWorld = new RigidBodyTransform();
      regionOne.getTransformToWorld(transformOneToWorld);

      Point2D[] mergedConcaveHullArray = new Point2D[mergedConcaveHull.size()];
      mergedConcaveHull.toArray(mergedConcaveHullArray);

      PlanarRegion planarRegion = new PlanarRegion(transformOneToWorld, mergedConcaveHullArray, newPolygonsFromConcaveHull);
      return planarRegion;
   }

   public static ArrayList<Point2D> mergeConcaveHulls(Point2D[] hullOne, Point2D[] hullTwo)
   {
      return mergeConcaveHulls(hullOne, hullTwo, null);
   }

   /**
    * Merges two ConcaveHulls. ConcaveHulls are assumed to be in clockwise order. Will fill holes.
    * Assumes that the two ConcaveHulls do intersect by some non-infinitesimal amount.
    * @param hullOne One hull to merge.
    * @param hullTwo The other hull to merge.
    * @return Merged hull.
    */
   public static ArrayList<Point2D> mergeConcaveHulls(Point2D[] hullOne, Point2D[] hullTwo, ConcaveHullMergerListener listener)
   {
//      printHull("hullOne", hullOne);
//      printHull("hullTwo", hullTwo);

      // Find a point that is guaranteed to be on the outside by finding one with the lowest x. 
      // In case of ties, first one should be fine
      Point2D[] workingHull = hullOne;
      Point2D[] restingHull = hullTwo;
      double minX = Double.POSITIVE_INFINITY;
      Point2D startingVertex = null;
      int workingHullIndex = -1;

      for (int i = 0; i < hullOne.length; i++)
      {
         Point2D vertex = hullOne[i];
         if (vertex.getX() < minX)
         {
            minX = vertex.getX();
            startingVertex = vertex;
            workingHullIndex = i;
         }
      }

      for (int i = 0; i < hullTwo.length; i++)
      {
         Point2D vertex = hullTwo[i];
         if (vertex.getX() < minX)
         {
            minX = vertex.getX();
            startingVertex = vertex;
            workingHullIndex = i;
            workingHull = hullTwo;
            restingHull = hullOne;
         }
      }

      if (listener != null)
      {
         listener.foundStartingVertexAndWorkingHull(startingVertex, workingHull, (workingHull == hullOne));
      }

      // Go in order until you hit an edge on the other convex hull or get to the end.
      ArrayList<Point2D> mergedVertices = new ArrayList<Point2D>();
      Point2D workingVertex = startingVertex;

      int nextIndex = (workingHullIndex + 1) % workingHull.length;

//      LogTools.info("startingVertex = " + startingVertex);
//
//      LogTools.info("STARTING!! workingHullIndex = " + workingHullIndex);
//      LogTools.info("nextIndex = " + nextIndex);

      int edgeStartIndexToSkip = -1;
      int count = 0;
      while (count < hullOne.length + hullTwo.length + 2)
      {
         count++;
         mergedVertices.add(workingVertex);

         Point2D nextVertex = workingHull[nextIndex];
         LineSegment2D workingEdge = new LineSegment2D(workingVertex, nextVertex);

         if (listener != null)
         {
            listener.consideringWorkingEdge(workingEdge, (workingHull == hullOne));
         }

         Pair<Integer, Point2D> intersection = findClosestIntersection(workingEdge, restingHull, edgeStartIndexToSkip);
         if (intersection == null)
         {
            workingVertex = nextVertex;

//            LogTools.info("workingVertex = " + workingVertex);

            if (workingVertex == startingVertex)
               break;

            nextIndex = (nextIndex + 1) % workingHull.length;
//            LogTools.info("nextIndex = " + nextIndex);

            edgeStartIndexToSkip = -1;
         }
         else
         {
            // If intersect an edge, take the path to the left.
            Point2D intersectionPoint = intersection.getRight();

            if (listener != null)
            {
               listener.foundIntersectionPoint(intersectionPoint, (workingHull == hullOne));
            }

            edgeStartIndexToSkip = nextIndex - 1;

            if (edgeStartIndexToSkip < 0)
            {
               edgeStartIndexToSkip = workingHull.length - 1;
            }

            Point2D[] temp = workingHull;
            workingHull = restingHull;
            restingHull = temp;

            nextIndex = intersection.getLeft();
            workingVertex = intersectionPoint;

//            LogTools.info("Swap. edgeStartIndexToSkip = " + edgeStartIndexToSkip + ", nextIndex = " + nextIndex);
         }

      }

      if (mergedVertices.size() > hullOne.length + hullTwo.length)
      {
         LogTools.error("mergedVertices.size() > hullOne.length + hullTwo.length. Something got looped!");
      }

      return mergedVertices;
   }

   private static void printHull(String name, Point2D[] hullPoints)
   {
      System.out.print(name + " = new double[][]{");

      for (int i = 0; i < hullPoints.length; i++)
      {
         Point2D point = hullPoints[i];

         System.out.println("{" + point.getX() + ", " + point.getY() + "}");
      }
      System.out.println("};");
   }

   /**
    * Finds the intersection in a concave hull as an edge crosses from outside to inside the hull.
    * If there are multiple intersections, returns the one that is closest to the edges start point.
    * If there is no intersection, returns null.
    * @param edge Edge to check for crossing.
    * @param concaveHull Concave Hull to check if edge crosses it.
    * @return Point2D where the hull crosses and the second index of the concave hull edge that it intersects.
    */
   public static Pair<Integer, Point2D> findClosestIntersection(LineSegment2D edge, Point2D[] concaveHull, int edgeStartIndexToSkip)
   {
      int previousIndex = concaveHull.length - 1;
      int nextIndex = 0;

      int bestIndex = -1;
      Point2DBasics bestIntersection = null;
      double bestDistanceSquared = Double.POSITIVE_INFINITY;

      while (nextIndex < concaveHull.length)
      {
         if (previousIndex != edgeStartIndexToSkip)
         {
            Point2D previousVertex = concaveHull[previousIndex];
            Point2D nextVertex = concaveHull[nextIndex];

            LineSegment2D hullEdge = new LineSegment2D(previousVertex, nextVertex);

            Point2DBasics intersection = edge.intersectionWith(hullEdge);

            if (intersection != null)
            {
//               System.out.println("Found edge. IndexToSkip = " + edgeStartIndexToSkip);
               //               if (edge.getFirstEndpoint().distanceSquared(intersection) > 1e-10)

               double distanceSquared = edge.getFirstEndpoint().distanceSquared(intersection);

               if (distanceSquared < bestDistanceSquared)
               {
                  bestDistanceSquared = distanceSquared;
                  bestIntersection = intersection;
                  bestIndex = nextIndex;
               }
            }
         }

         previousIndex = nextIndex;
         nextIndex++;
      }

      if (bestIntersection == null)
         return null;

      return new ImmutablePair<Integer, Point2D>(bestIndex, new Point2D(bestIntersection));

   }
}
