package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.commons.lists.ListWrappingIndexTools.getNext;
import static us.ihmc.commons.lists.ListWrappingIndexTools.getPrevious;
import static us.ihmc.commons.lists.ListWrappingIndexTools.getWrap;
import static us.ihmc.commons.lists.ListWrappingIndexTools.next;
import static us.ihmc.commons.lists.ListWrappingIndexTools.previous;
import static us.ihmc.commons.lists.ListWrappingIndexTools.removeAllExclusive;
import static us.ihmc.commons.lists.ListWrappingIndexTools.subLengthInclusive;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.computeConcaveHullPocket;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.findClosestIntersectionWithRay;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.findInnerClosestEdgeToVertex;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.isConvexAtVertex;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.isVertexPreventingKink;

import java.util.List;

import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * This class gathers different filters on a concave hull. The filters aim to reduce the hull
 * complexity by removing less significant vertices. The filters gathered here only tend to reduce
 * the area of the concave hull.
 * 
 * @author Sylvain
 */
public class ConcaveHullPruningFilteringTools
{
   /**
    * Filter out vertices that create "peaks" or barely stick out the line described by the previous
    * and next vertices. Peaks are identified by a threshold on the angle between two consecutive
    * edges. Only convex peaks or shallow angles are removed, meaning this filter only reduces the area
    * of the concave hull.
    * 
    * @param shallowAngleThreshold         should be a small positive angle in radians. 0 will not
    *                                      remove any vertex.
    * @param peakAngleThreshold            should be close to {@link Math#PI}.
    * @param concaveHullCollectionToFilter the collection of concave hulls to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutPeaksAndShallowAngles(double shallowAngleThreshold, double peakAngleThreshold,
                                                    ConcaveHullCollection concaveHullCollectionToFilter)
   {
      int totalNumberOfVerticesRemoved = 0;
      int vertexRemoved = 0;
      int counter = 0;
      do
      {
         if (counter++ == 5)
            System.out.println();

         vertexRemoved = 0;
         for (ConcaveHull concaveHullToFilter : concaveHullCollectionToFilter)
            vertexRemoved += filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullToFilter.getConcaveHullVertices());

         System.out.println("filtered vertices " + vertexRemoved);
         totalNumberOfVerticesRemoved += vertexRemoved;
      }
      while (vertexRemoved != 0);
      return totalNumberOfVerticesRemoved;
   }

   /**
    * Filter out vertices that create "peaks" or barely stick out the line described by the previous
    * and next vertices. Peaks are identified by a threshold on the angle between two consecutive
    * edges. Only convex peaks or shallow angles are removed, meaning this filter only reduces the area
    * of the concave hull.
    * 
    * @param shallowAngleThreshold should be a small positive angle in radians. 0 will not remove any
    *                              vertex.
    * @param peakAngleThreshold    should be close to {@link Math#PI}.
    * @param concaveHullToFilter   the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutPeaksAndShallowAngles(double shallowAngleThreshold, double peakAngleThreshold, ConcaveHull concaveHullToFilter)
   {
      return filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullToFilter.getConcaveHullVertices());
   }

   /**
    * Filter out vertices that create "peaks" or barely stick out the line described by the previous
    * and next vertices. Peaks are identified by a threshold on the angle between two consecutive
    * edges. Only convex peaks or shallow angles are removed, meaning this filter only reduces the area
    * of the concave hull.
    * 
    * @param shallowAngleThreshold       should be a small positive angle in radians. 0 will not remove
    *                                    any vertex.
    * @param peakAngleThreshold          should be close to {@link Math#PI}.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutPeaksAndShallowAngles(double shallowAngleThreshold, double peakAngleThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      shallowAngleThreshold = -Math.abs(shallowAngleThreshold);
      peakAngleThreshold = -Math.abs(peakAngleThreshold);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         if (!ConcaveHullTools.isConvexAtVertex(currentIndex, concaveHullVerticesToFilter))
         {
            currentIndex++;
            continue;
         }

         double angle = ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(currentIndex, concaveHullVerticesToFilter);
         boolean isAngleWithinThresholds = angle < peakAngleThreshold || angle > shallowAngleThreshold;

         if (isAngleWithinThresholds && !isVertexPreventingKink(currentIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(currentIndex);
            numberOfVerticesRemoved++;
         }
         else
         {
            currentIndex++;
         }
      }
      return numberOfVerticesRemoved;
   }

   public static int filterOutShallowVertices(double percentageThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2D a = concaveHullVerticesToFilter.get(0);
      Point2D b = concaveHullVerticesToFilter.get(1);
      Point2D c = concaveHullVerticesToFilter.get(2);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // If convex at b, then b should be on the outside of ac => on the left of the vector ac.
         boolean isConvex = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(b, a, c);

         int nextIndex = next(currentIndex, concaveHullVerticesToFilter);
         if (isConvex && a.distance(c) / (a.distance(b) + b.distance(c)) > percentageThreshold
               && !isVertexPreventingKink(nextIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(nextIndex);
            b = c;
            numberOfVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            currentIndex++;
         }

         c = getNext(nextIndex, concaveHullVerticesToFilter);
      }

      return numberOfVerticesRemoved;
   }

   public static int filterOutGroupsOfShallowVertices(double percentageThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      Point2D firstVertex;
      Point2D intermediateVertex;
      Point2D lastVertex;

      double perimeter = 0.0;
      double cuttingDistance = 0.0;
      double ratio = 0.0;

      for (int startCutIndex = 0; startCutIndex < concaveHullVerticesToFilter.size();)
      {
         firstVertex = concaveHullVerticesToFilter.get(startCutIndex);
         int intermediateIndex = next(startCutIndex, concaveHullVerticesToFilter);
         intermediateVertex = concaveHullVerticesToFilter.get(intermediateIndex);
         int lastVertexIndex = next(intermediateIndex, concaveHullVerticesToFilter);
         lastVertex = concaveHullVerticesToFilter.get(lastVertexIndex);

         perimeter = firstVertex.distance(intermediateVertex);
         perimeter += intermediateVertex.distance(lastVertex);
         cuttingDistance = firstVertex.distance(lastVertex);

         ratio = cuttingDistance / perimeter;

         // Cannot cut, skipping to the next vertex
         if (ratio < percentageThreshold)
         {
            startCutIndex++;
            continue;
         }

         // Can cut at least one vertex, going further to see if more can be removed
         int endCutIndex = lastVertexIndex;

         while (ratio > percentageThreshold)
         {
            intermediateVertex = lastVertex;
            endCutIndex = lastVertexIndex;
            lastVertexIndex = next(lastVertexIndex, concaveHullVerticesToFilter);
            lastVertex = concaveHullVerticesToFilter.get(lastVertexIndex);

            perimeter += intermediateVertex.distance(lastVertex);
            cuttingDistance = firstVertex.distance(lastVertex);

            ratio = cuttingDistance / perimeter;
         }

         int firstRemovableVertexIndex = next(startCutIndex, concaveHullVerticesToFilter);

         for (int checkIndex = firstRemovableVertexIndex; checkIndex != endCutIndex;)
         {
            Point2D vertexToCheck = concaveHullVerticesToFilter.get(checkIndex);

            if (!EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(vertexToCheck, firstVertex, lastVertex))
            {
               // Reducing the cutting line
               endCutIndex = checkIndex;
               if (endCutIndex == startCutIndex)
               { // Nothing to cut anymore
                  break;
               }
               // Starting over the check
               checkIndex = startCutIndex;
            }
            else
            {
               checkIndex = next(checkIndex, concaveHullVerticesToFilter);
            }
         }

         // Nothing to cut, skipping.
         if (endCutIndex == startCutIndex)
         {
            startCutIndex++;
            continue;
         }

         // Removing the vertices.
         numberOfVerticesRemoved += removeAllExclusive(startCutIndex, endCutIndex, concaveHullVerticesToFilter);
      }

      return numberOfVerticesRemoved;
   }

   /**
    * By looking at each triplet of successive vertices, a triangle is formed. The area of the triangle
    * is computed and if below the given threshold, the middle vertex is removed.
    * 
    * @param areaThreshold               a vertex forming a triangle with an area smaller than that
    *                                    parameter will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutSmallTriangles(double areaThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2D a = concaveHullVerticesToFilter.get(0);
      Point2D b = concaveHullVerticesToFilter.get(1);
      Point2D c = concaveHullVerticesToFilter.get(2);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // If convex at b, then b should be on the outside of ac => on the left of the vector ac.
         boolean isConvex = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(b, a, c);
         int nextIndex = next(currentIndex, concaveHullVerticesToFilter);
         if (isConvex && EuclidGeometryTools.triangleArea(a, b, c) < areaThreshold && !isVertexPreventingKink(nextIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(nextIndex);
            b = c;
            numberOfVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get(next(nextIndex, concaveHullVerticesToFilter));
      }

      return numberOfVerticesRemoved;
   }

   public static int flattenShallowPockets(double depthThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      ConcaveHullPocket pocket = new ConcaveHullPocket();

      Vector2D shift = new Vector2D();

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size(); currentIndex++)
      {
         if (isConvexAtVertex(currentIndex, concaveHullVerticesToFilter))
            continue;

         boolean success = computeConcaveHullPocket(currentIndex, pocket, concaveHullVerticesToFilter);
         if (!success)
            continue;

         double maxDepth = pocket.getMaxDepth();
         if (maxDepth > depthThreshold)
            continue;

         Point2D startBridgeVertex = new Point2D(pocket.getStartBridgeVertex());
         Point2D endBridgeVertex = new Point2D(pocket.getEndBridgeVertex());
         shift.sub(endBridgeVertex, startBridgeVertex);
         shift.normalize();
         // Rotate to the right
         shift.set(shift.getY(), -shift.getX());
         shift.scale(maxDepth);

         startBridgeVertex.add(shift);
         endBridgeVertex.add(shift);

         int startBridgeVertexIndex = pocket.getStartBridgeIndex();
         int endBridgeVertexIndex = pocket.getEndBridgeIndex();
         numberOfVerticesRemoved += removeAllExclusive(startBridgeVertexIndex, endBridgeVertexIndex, concaveHullVerticesToFilter);
      }

      return numberOfVerticesRemoved;
   }

   /**
    * Removes vertices to filter short edges. Only convex vertices are removed, meaning the polygon
    * area can only decrease when calling this method.
    * 
    * @param concaveAngleLimit           threshold to define a concavity. 0 rad being flat, negative
    *                                    convex, positive concave.
    * @param lengthThreshold             any edge shorter than that will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutShortEdges(double lengthThreshold, ConcaveHullCollection concaveHullCollectionToFilter)
   {
      int numberOfRemovedVertices = 0;
      for (ConcaveHull concaveHullToFilter : concaveHullCollectionToFilter)
         numberOfRemovedVertices += filterOutShortEdges(lengthThreshold, concaveHullToFilter.getConcaveHullVertices());
      return numberOfRemovedVertices;
   }

   /**
    * Removes vertices to filter short edges. Only convex vertices are removed, meaning the polygon
    * area can only decrease when calling this method.
    * 
    * @param concaveAngleLimit           threshold to define a concavity. 0 rad being flat, negative
    *                                    convex, positive concave.
    * @param lengthThreshold             any edge shorter than that will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutShortEdges(double lengthThreshold, ConcaveHull concaveHullToFilter)
   {
      return filterOutShortEdges(lengthThreshold, concaveHullToFilter.getConcaveHullVertices());
   }

   /**
    * Removes vertices to filter short edges. Only convex vertices are removed, meaning the polygon
    * area can only decrease when calling this method.
    * 
    * @param concaveAngleLimit           threshold to define a concavity. 0 rad being flat, negative
    *                                    convex, positive concave.
    * @param lengthThreshold             any edge shorter than that will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutShortEdges(double lengthThreshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      double lengthThresholdSquared = lengthThreshold * lengthThreshold;

      Vector2D edgeVector = new Vector2D();
      Vector2D previousEdgeVector = new Vector2D();
      Vector2D nextEdgeVector = new Vector2D();

      for (int beforeEdgeVertexIndex = 0; beforeEdgeVertexIndex < concaveHullVerticesToFilter.size();)
      {
         int firstEdgeVertexIndex = next(beforeEdgeVertexIndex, concaveHullVerticesToFilter);
         int secondEdgeVertexIndex = next(firstEdgeVertexIndex, concaveHullVerticesToFilter);
         int afterEdgeVertexIndex = next(secondEdgeVertexIndex, concaveHullVerticesToFilter);

         Point2D firstEdgeVertex = concaveHullVerticesToFilter.get(firstEdgeVertexIndex);
         Point2D secondEdgeVertex = concaveHullVerticesToFilter.get(secondEdgeVertexIndex);
         double edgeLengthSquared = firstEdgeVertex.distanceSquared(secondEdgeVertex);

         // This is a long edge, skip.
         if (edgeLengthSquared > lengthThresholdSquared)
         {
            beforeEdgeVertexIndex++;
            continue;
         }

         Point2D beforeEdgeVertex = concaveHullVerticesToFilter.get(beforeEdgeVertexIndex);
         Point2D afterEdgeVertex = concaveHullVerticesToFilter.get(afterEdgeVertexIndex);

         boolean isFirstEdgeVertexConvex = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(firstEdgeVertex, beforeEdgeVertex, secondEdgeVertex);
         boolean isSecondEdgeVertexConvex = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(secondEdgeVertex, firstEdgeVertex, afterEdgeVertex);

         // Both vertices are in a concavity, cannot remove any of them without expanding the concave hull.
         if (!isFirstEdgeVertexConvex && !isSecondEdgeVertexConvex)
         {
            beforeEdgeVertexIndex++;
         }
         // Only one of the two vertices can be removed 
         else if (isFirstEdgeVertexConvex != isSecondEdgeVertexConvex)
         {
            // Is it the first one?
            if (isFirstEdgeVertexConvex)
            {
               // Check if removing the vertex would create a kink in the polygon
               if (isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Skip vertex
                  beforeEdgeVertexIndex++;
               }
               else
               { // It is safe to remove the vertex
                  concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
            }
            // Reaching here means only the second vertex could be removed. Check if removing it would create a kink.
            else
            {
               // Check if removing the vertex would create a kink in the polygon
               if (isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Skip vertex
                  beforeEdgeVertexIndex++;
               }
               else
               { // Safe to remove the second vertex 
                  concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
            }
         }
         else // Here, any of the two vertices can be removed.
         {
            edgeVector.sub(secondEdgeVertex, firstEdgeVertex);
            edgeVector.normalize();

            previousEdgeVector.sub(firstEdgeVertex, beforeEdgeVertex);
            previousEdgeVector.normalize();

            nextEdgeVector.sub(afterEdgeVertex, secondEdgeVertex);
            nextEdgeVector.normalize();

            double edgeDotNextEdge = edgeVector.dot(nextEdgeVector);
            double previousEdgeDotEdge = previousEdgeVector.dot(edgeVector);

            // Pick the vertex that affects the least the polygon when removing it.
            // Basically looking at which vertex is the "flatter"
            if (previousEdgeDotEdge > edgeDotNextEdge)
            { // The first vertex is the best option. Still need to check if removing it would create a kink.
               if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Can remove the first vertex
                  concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
               else
               { // Cannot remove the first vertex. Check if the second vertex can be removed.
                  if (!isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the second vertex.
                     concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                     numberOfVerticesRemoved++;
                  }
                  else
                  { // Cannot removed the second vertex either, skip.
                     beforeEdgeVertexIndex++;
                  }
               }
            }
            else
            { // The second vertex is the best option, checking if removing it would create a kink.
               if (!isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Can remove the second vertex
                  concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
               else
               { // Cannot remove the second vertex. Check if the first vertex can be removed.
                  if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the first vertex.
                     concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                     numberOfVerticesRemoved++;
                  }
                  else
                  { // Cannot removed the first vertex either, skip.
                     beforeEdgeVertexIndex++;
                  }
               }
            }
         }
      }
      return numberOfVerticesRemoved;
   }

   public static int filterByRay(double threshold, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      Vector2D rayDirection = new Vector2D();
      Point2D intersection = new Point2D();
      double thresholdSquared = threshold * threshold;

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size(); currentIndex++)
      {
         Point2D rayOrigin = concaveHullVerticesToFilter.get(currentIndex);
         rayDirection.sub(getNext(currentIndex, concaveHullVerticesToFilter), getPrevious(currentIndex, concaveHullVerticesToFilter));
         rayDirection.set(rayDirection.getY(), -rayDirection.getX());

         int startSearchIndex = next(currentIndex, concaveHullVerticesToFilter);
         int endSearchIndex = previous(currentIndex, concaveHullVerticesToFilter);
         int edgeFirstVertexIndex = findClosestIntersectionWithRay(rayOrigin,
                                                                   rayDirection,
                                                                   startSearchIndex,
                                                                   endSearchIndex,
                                                                   concaveHullVerticesToFilter,
                                                                   intersection);

         if (rayOrigin.distanceSquared(intersection) < thresholdSquared)
         {
            int edgeSecondVertexIndex = next(edgeFirstVertexIndex, concaveHullVerticesToFilter);
            int firstSubLength = subLengthInclusive(currentIndex, edgeFirstVertexIndex, concaveHullVerticesToFilter);
            int secondSubLength = subLengthInclusive(edgeSecondVertexIndex, currentIndex, concaveHullVerticesToFilter);

            if (firstSubLength < secondSubLength)
            {
               concaveHullVerticesToFilter.get(edgeFirstVertexIndex).set(intersection);
               numberOfVerticesRemoved += removeAllExclusive(currentIndex, edgeFirstVertexIndex, concaveHullVerticesToFilter);
            }
            else
            {
               concaveHullVerticesToFilter.get(edgeSecondVertexIndex).set(intersection);
               numberOfVerticesRemoved += removeAllExclusive(edgeSecondVertexIndex, currentIndex, concaveHullVerticesToFilter);
            }
         }
      }
      return numberOfVerticesRemoved;
   }

   public static ConcaveHullCollection filterOutNarrowRegions(double alphaRadius, ConcaveHullCollection concaveHullCollectionToFilter)
   {
      ConcaveHullCollection filteredConcaveHullCollection = new ConcaveHullCollection();

      for (ConcaveHull concaveHull : concaveHullCollectionToFilter)
      {
         filteredConcaveHullCollection.addAll(filterOutNarrowRegions(alphaRadius, concaveHull));
      }

      return filteredConcaveHullCollection;
   }

   public static ConcaveHullCollection filterOutNarrowRegions(double alphaRadius, ConcaveHull concaveHullToFilter)
   {
      int numberOfVertices = concaveHullToFilter.getNumberOfVertices();

      for (int iEdgeStartA = 0, iEdgeEndA = 1; iEdgeStartA < numberOfVertices - 1; iEdgeStartA++, iEdgeEndA++)
      {
         Point2D edgeStartA = concaveHullToFilter.getVertex(iEdgeStartA);
         Point2D edgeEndA = concaveHullToFilter.getVertex(iEdgeEndA);

         for (int iEdgeStartB = iEdgeEndA, iEdgeEndB = iEdgeStartB + 1; iEdgeStartB < numberOfVertices; iEdgeStartB++, iEdgeEndB++)
         {
            iEdgeEndB %= numberOfVertices;

            Point2D edgeStartB = concaveHullToFilter.getVertex(iEdgeStartB);
            Point2D edgeEndB = concaveHullToFilter.getVertex(iEdgeEndB);

            if (perimeterDistanceBetweenVertices(iEdgeEndA, iEdgeStartB, concaveHullToFilter) <= 2.0 * alphaRadius)
               continue;
            if (perimeterDistanceBetweenVertices(iEdgeEndB, iEdgeStartA, concaveHullToFilter) <= 2.0 * alphaRadius)
               continue;

            double distance = EuclidCoreMissingTools.distanceBetweenTwoLineSegment2Ds(edgeStartA, edgeEndA, edgeStartB, edgeEndB);

            if (distance > 2.0 * alphaRadius)
               continue;

            Point2D closestPointOnEdgeA = new Point2D();
            Point2D closestPointOnEdgeB = new Point2D();
            EuclidCoreMissingTools.closestPoint2DsBetweenTwoLineSegment2Ds(edgeStartA,
                                                                           edgeEndA,
                                                                           edgeStartB,
                                                                           edgeEndB,
                                                                           closestPointOnEdgeA,
                                                                           closestPointOnEdgeB);

            // Splitting the hull into 2:
            List<Point2D> concaveHullVertices = concaveHullToFilter.getConcaveHullVertices();
            ConcaveHull subConcaveHullA = new ConcaveHull(ListWrappingIndexTools.subListInclusive(iEdgeEndA, iEdgeStartB, concaveHullVertices));
            ConcaveHull subConcaveHullB = new ConcaveHull(ListWrappingIndexTools.subListInclusive(iEdgeEndB, iEdgeStartA, concaveHullVertices));

            Point2D newVertexAForSubHullA = new Point2D();
            Point2D newVertexAForSubHullB = new Point2D();

            // The closest point has to either be edgeStartA or edgeEndA
            if (EuclidGeometryTools.percentageAlongLineSegment2D(closestPointOnEdgeA, edgeStartA, edgeEndA) < 0.5)
            { // Popping the vertex at iEdgeStartA
               computeSeparatedPointsAroundVertex(2.0 * alphaRadius, iEdgeStartA, concaveHullVertices, newVertexAForSubHullB, newVertexAForSubHullA);
               subConcaveHullA.getConcaveHullVertices().add(0, newVertexAForSubHullA);
               subConcaveHullB.getConcaveHullVertices().set(subConcaveHullB.getNumberOfVertices() - 1, newVertexAForSubHullB);
            }
            else
            { // Popping the vertex at iEdgeEndA
               computeSeparatedPointsAroundVertex(2.0 * alphaRadius, iEdgeEndA, concaveHullVertices, newVertexAForSubHullB, newVertexAForSubHullA);
               subConcaveHullA.getConcaveHullVertices().set(0, newVertexAForSubHullA);
               subConcaveHullB.getConcaveHullVertices().add(newVertexAForSubHullB);
            }

            Point2D newVertexBForSubHullA = new Point2D();
            Point2D newVertexBForSubHullB = new Point2D();

            // The closest point has to either be edgeStartB or edgeEndB
            if (EuclidGeometryTools.percentageAlongLineSegment2D(closestPointOnEdgeB, edgeStartB, edgeEndB) < 0.5)
            { // Popping the vertex at iEdgeStartB
               computeSeparatedPointsAroundVertex(2.0 * alphaRadius, iEdgeStartB, concaveHullVertices, newVertexBForSubHullA, newVertexBForSubHullB);
               subConcaveHullA.getConcaveHullVertices().set(subConcaveHullA.getNumberOfVertices() - 1, newVertexBForSubHullA);
               subConcaveHullB.getConcaveHullVertices().add(0, newVertexBForSubHullB);
            }
            else
            { // Popping the vertex at iEdgeEndB
               computeSeparatedPointsAroundVertex(2.0 * alphaRadius, iEdgeEndB, concaveHullVertices, newVertexBForSubHullA, newVertexBForSubHullB);
               subConcaveHullA.getConcaveHullVertices().add(newVertexBForSubHullA);
               subConcaveHullB.getConcaveHullVertices().set(0, newVertexBForSubHullB);
            }

            ConcaveHullCollection output = new ConcaveHullCollection();
            output.addAll(filterOutNarrowRegions(alphaRadius, subConcaveHullA));
            output.addAll(filterOutNarrowRegions(alphaRadius, subConcaveHullB));
            return output;
         }
      }

      // If we reached here, this indicates that the concave hull was not filtered.
      return new ConcaveHullCollection(concaveHullToFilter);
   }

   /**
    * Computes the coordinates of 2 points <tt>A</tt> and <tt>B</tt> that lie on the previous and next
    * edge of the vertex <tt>V</tt> such that the 2 points at a distance {@code separationDistance}
    * from each other.
    * <p>
    * Uses the inherent property of the problem: the triangle AVB is isosceles with |AV| = |VB| and
    * |AB| = {@code separationDistance}. Computing &theta;, the inner triangle angle at the vertex V,
    * the distance |AV| = |VB| = {@code separationDistance} / (2 * sin(&theta;)).
    * </p>
    * 
    * @param separationDistance        the distance that A and B should be from each other.
    * @param vertexIndex               the index of the vertex <tt>V</tt>.
    * @param concaveHullVertices       the vertices of the concave hull to which the vertex <tt>V</tt>
    *                                  belongs to. Not modified.
    * @param pointOnPreviousEdgeToPack the point in which the coordinates of the point <tt>A</tt> are
    *                                  stored. Modified.
    * @param pointOnNextEdgeToPack     the point in which the coordinates of the point <tt>B</tt> are
    *                                  stored. Modified.
    */
   private static void computeSeparatedPointsAroundVertex(double separationDistance, int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices,
                                                          Point2DBasics pointOnPreviousEdgeToPack, Point2DBasics pointOnNextEdgeToPack)
   {
      UnitVector2D directionToNext = new UnitVector2D();
      UnitVector2D directionToPrevious = new UnitVector2D();

      Point2DReadOnly vertex = getWrap(vertexIndex, concaveHullVertices);
      directionToNext.sub(getNext(vertexIndex, concaveHullVertices), vertex);
      directionToPrevious.sub(getPrevious(vertexIndex, concaveHullVertices), getWrap(vertexIndex, concaveHullVertices));

      double angleAtVertex = Math.abs(directionToPrevious.angle(directionToNext));

      if (angleAtVertex > Math.PI)
         angleAtVertex = 2.0 * Math.PI - angleAtVertex;

      double distanceAlongEdge = 0.5 * separationDistance / Math.sin(angleAtVertex);

      pointOnPreviousEdgeToPack.scaleAdd(distanceAlongEdge, directionToPrevious, vertex);
      pointOnNextEdgeToPack.scaleAdd(distanceAlongEdge, directionToNext, vertex);
   }

   private static double perimeterDistanceBetweenVertices(int firstVertexIndex, int secondVertexIndex, ConcaveHull concaveHull)
   {
      List<Point2D> pathToWalk = ListWrappingIndexTools.subListInclusive(firstVertexIndex, secondVertexIndex, concaveHull.getConcaveHullVertices());

      double perimeterDistance = 0.0;

      for (int vertexIndex = 0; vertexIndex < pathToWalk.size() - 1; vertexIndex++)
      {
         perimeterDistance += pathToWalk.get(vertexIndex).distance(pathToWalk.get(vertexIndex + 1));
      }

      return perimeterDistance;
   }

   public static int innerAlphaShapeFiltering(double alpha, int deadIndexRegion, List<Point2D> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      Point2D closestPoint = new Point2D();
      double alphaSquared = alpha * alpha;

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size(); currentIndex++)
      {
         Point2D currentVertex = concaveHullVerticesToFilter.get(currentIndex);
         int closestEdgeFirstVertexIndex = findInnerClosestEdgeToVertex(currentIndex, deadIndexRegion, concaveHullVerticesToFilter, closestPoint);

         if (closestPoint.distanceSquared(currentVertex) < alphaSquared)
         {
            int closestEdgeSecondVertexIndex = next(closestEdgeFirstVertexIndex, concaveHullVerticesToFilter);
            int firstSubLength = subLengthInclusive(currentIndex, closestEdgeFirstVertexIndex, concaveHullVerticesToFilter);
            int secondSubLength = subLengthInclusive(closestEdgeSecondVertexIndex, currentIndex, concaveHullVerticesToFilter);

            if (firstSubLength <= 1 || secondSubLength <= 1)
            {
               continue;
            }

            if (firstSubLength <= secondSubLength)
            {
               concaveHullVerticesToFilter.get(closestEdgeFirstVertexIndex).set(closestPoint);
               numberOfVerticesRemoved += removeAllExclusive(currentIndex, closestEdgeFirstVertexIndex, concaveHullVerticesToFilter);
            }
            else
            {
               concaveHullVerticesToFilter.get(closestEdgeSecondVertexIndex).set(closestPoint);
               numberOfVerticesRemoved += removeAllExclusive(closestEdgeSecondVertexIndex, currentIndex, concaveHullVerticesToFilter);
            }
         }
      }

      return numberOfVerticesRemoved;
   }

}
