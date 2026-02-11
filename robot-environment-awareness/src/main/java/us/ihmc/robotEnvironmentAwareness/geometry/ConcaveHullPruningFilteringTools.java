package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.commons.lists.ListWrappingIndexTools.getNext;
import static us.ihmc.commons.lists.ListWrappingIndexTools.getPrevious;
import static us.ihmc.commons.lists.ListWrappingIndexTools.next;
import static us.ihmc.commons.lists.ListWrappingIndexTools.previous;
import static us.ihmc.commons.lists.ListWrappingIndexTools.removeAllExclusive;
import static us.ihmc.commons.lists.ListWrappingIndexTools.subLengthInclusive;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.computeConcaveHullPocket;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.findClosestIntersectionWithRay;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.isConvexAtVertex;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.isVertexPreventingKink;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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
}
