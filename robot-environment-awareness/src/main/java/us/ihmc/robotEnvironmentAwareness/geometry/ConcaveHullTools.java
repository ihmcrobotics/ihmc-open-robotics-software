package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.lists.ListWrappingIndexTools.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class ConcaveHullTools
{
   /**
    * Returns true only if removing the vertex would generate a kink in the concave polygon.
    * Meaning, it would cause several edges to cross each other.
    */
   public static boolean isVertexPreventingKink(int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      int vertexPreviousIndex = previous(vertexIndex, concaveHullVertices);
      vertexIndex %= concaveHullVertices.size();
      int vertexNextIndex = next(vertexIndex, concaveHullVertices);

      Point2DReadOnly a = concaveHullVertices.get(vertexPreviousIndex);
      Point2DReadOnly b = concaveHullVertices.get(vertexIndex);
      Point2DReadOnly c = concaveHullVertices.get(vertexNextIndex);

      int currentIndex = next(vertexNextIndex, concaveHullVertices);

      while (currentIndex != vertexPreviousIndex)
      {
         if (EuclidGeometryTools.isPoint2DInsideTriangleABC(concaveHullVertices.get(currentIndex), a, b, c))
            return true;
         currentIndex = next(currentIndex, concaveHullVertices);
      }

      return false;
   }

   public static boolean isClockwiseOrdered(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      double sumOfAngles = 0.0;

      Vector2D previousEdge = new Vector2D();
      Vector2D nextEdge = new Vector2D();

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.size(); vertexIndex++)
      {
         int previousVertexIndex = previous(vertexIndex, concaveHullVertices);
         int nextVertexIndex = next(vertexIndex, concaveHullVertices);

         Point2DReadOnly previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
         Point2DReadOnly nextVertex = concaveHullVertices.get(nextVertexIndex);

         previousEdge.sub(vertex, previousVertex);
         nextEdge.sub(nextVertex, vertex);
         sumOfAngles += previousEdge.angle(nextEdge);
      }

      return sumOfAngles <= 0.0;
   }

   public static void ensureClockwiseOrdering(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      if (!isClockwiseOrdered(concaveHullVertices))
         Collections.reverse(concaveHullVertices);
   }

   public static void ensureCounterClockwiseOrdering(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      if (isClockwiseOrdered(concaveHullVertices))
         Collections.reverse(concaveHullVertices);
   }

   public static double computePerimeter(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      double perimeter = 0.0;
      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Point2DReadOnly vertex = concaveHullVertices.get(i);
         Point2DReadOnly nextVertex = concaveHullVertices.get(next(i, concaveHullVertices));
         perimeter += vertex.distance(nextVertex);
      }
      return perimeter;
   }

   public static int removeSuccessiveDuplicateVertices(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      int numberOfVerticesRemoved = 0;

      for (int index = 0; index < concaveHullVertices.size();)
      {
         int nextIndex = next(index, concaveHullVertices);
         if (concaveHullVertices.get(index).equals(concaveHullVertices.get(nextIndex)))
         {
            concaveHullVertices.remove(nextIndex);
            numberOfVerticesRemoved++;
         }
         else
         {
            index++;
         }
      }
      return numberOfVerticesRemoved;
   }

   public static boolean computeConcaveHullPocket(int concaveVertexIndex, ConcaveHullPocket pocketToPack, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      pocketToPack.clear();
      boolean success = findBridgeVertices(concaveVertexIndex, pocketToPack, concaveHullVertices);
      if (!success)
      {
         pocketToPack.clear();
         return false;
      }
      success = findDeepestVertexInPocket(pocketToPack, concaveHullVertices);
      if (!success)
      {
         pocketToPack.clear();
         return false;
      }
      return true;
   }

   public static Set<ConcaveHullPocket> findConcaveHullPockets(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold)
   {
      Set<ConcaveHullPocket> pockets = new HashSet<>();

      int startIndex = 0;

      while (startIndex < concaveHullVertices.size())
      {
         ConcaveHullPocket newPocket = findFirstConcaveHullPocket(concaveHullVertices, startIndex);
         if (newPocket == null)
            break;

         if (newPocket.getMaxDepth() >= depthThreshold)
         {
            if (!pockets.add(newPocket))
               break;
         }

         startIndex = newPocket.getEndBridgeIndex() + 1;
      }

      return pockets;
   }

   public static ConcaveHullPocket findFirstConcaveHullPocket(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      return findFirstConcaveHullPocket(concaveHullVertices, 0);
   }

   public static ConcaveHullPocket findFirstConcaveHullPocket(List<? extends Point2DReadOnly> concaveHullVertices, int startIndex)
   {
      if (startIndex < 0 || startIndex >= concaveHullVertices.size())
         throw new IndexOutOfBoundsException("Expected startIndex in [0, " + concaveHullVertices.size() + "[, received: " + startIndex);
      ConcaveHullPocket pocket = null;

      for (int i = startIndex; i < concaveHullVertices.size() && pocket == null; i++)
         pocket = ConcaveHullTools.computeConcaveHullPocket(i, concaveHullVertices);

      return pocket;
   }

   public static ConcaveHullPocket computeConcaveHullPocket(int concaveVertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      ConcaveHullPocket pocketToReturn = new ConcaveHullPocket();
      boolean success = findBridgeVertices(concaveVertexIndex, pocketToReturn, concaveHullVertices);
      if (!success)
         return null;
      success = findDeepestVertexInPocket(pocketToReturn, concaveHullVertices);
      if (!success)
         return null;
      return pocketToReturn;
   }

   /**
    * Find the two vertices that forms a bridge over the pocket that the concaveVertex belongs to.
    * 
    * @param concaveVertexIndex
    * @param pocketToPack
    * @param concaveHullVertices
    * @return {firstBridgeIndex, secondBridgeIndex} or null if the polygon is actually convex at the
    *         given vertex.
    */
   public static boolean findBridgeVertices(int concaveVertexIndex, ConcaveHullPocket pocketToPack, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      concaveVertexIndex %= concaveHullVertices.size();
      Point2DReadOnly concaveVertex = concaveHullVertices.get(concaveVertexIndex);

      int firstBridgeIndex = previous(concaveVertexIndex, concaveHullVertices);
      int secondBridgeIndex = next(concaveVertexIndex, concaveHullVertices);

      Point2DReadOnly firstBridgeVertex = concaveHullVertices.get(firstBridgeIndex);
      Point2DReadOnly secondBridgeVertex = concaveHullVertices.get(secondBridgeIndex);

      // The polygon is convex at this vertex => no pocket => no bridge
      if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(concaveVertex, firstBridgeVertex, secondBridgeVertex))
         return false;

      int startIndexCandidate = firstBridgeIndex;
      int endIndexCandidate = secondBridgeIndex;

      // Doing a loop with the start and end indices going opposite ways at the same time.
      while (true)
      {
         startIndexCandidate = previous(startIndexCandidate, concaveHullVertices);
         // Check if all the vertices have been explored
         if (startIndexCandidate == endIndexCandidate)
            break;

         endIndexCandidate = next(endIndexCandidate, concaveHullVertices);
         // Check if all the vertices have been explored
         if (startIndexCandidate == endIndexCandidate)
            break;

         Point2DReadOnly startCandidate = concaveHullVertices.get(startIndexCandidate);
         Point2DReadOnly endCandidate = concaveHullVertices.get(endIndexCandidate);

         if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(startCandidate, firstBridgeVertex, secondBridgeVertex))
         { // startIndexCandidate is a potential firstBridgeIndex.
            boolean isBridgeCoveringPocket = true;

            // Make sure that the new bridge would go over all the pocket vertices
            for (int i = next(startIndexCandidate, concaveHullVertices); i != secondBridgeIndex && isBridgeCoveringPocket; i = next(i, concaveHullVertices))
               isBridgeCoveringPocket = !EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(concaveHullVertices.get(i), startCandidate, secondBridgeVertex);

            if (isBridgeCoveringPocket)
            {
               firstBridgeIndex = startIndexCandidate;
               firstBridgeVertex = startCandidate;
               // Reset the other index to rescan the vertices
               endIndexCandidate = secondBridgeIndex;
            }
         }
         else if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(endCandidate, firstBridgeVertex, secondBridgeVertex))
         { // endIndexCandidate is the new secondBridgeIndex.
            boolean isBridgeCoveringPocket = true;

            // Make sure that the new bridge would go over all the pocket vertices
            for (int i = next(firstBridgeIndex, concaveHullVertices); i != endIndexCandidate && isBridgeCoveringPocket; i = next(i, concaveHullVertices))
               isBridgeCoveringPocket = !EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(concaveHullVertices.get(i), firstBridgeVertex, endCandidate);

            if (isBridgeCoveringPocket)
            {
               secondBridgeIndex = endIndexCandidate;
               secondBridgeVertex = endCandidate;
               // Reset the other index to rescan the vertices
               startIndexCandidate = firstBridgeIndex;
            }
         }
      }

      pocketToPack.setBridgeIndices(firstBridgeIndex, secondBridgeIndex);
      pocketToPack.setBridgeVertices(firstBridgeVertex, secondBridgeVertex);

      return true;
   }

   /**
    * Find the deepest vertex in the given pocket. It updates {@link ConcaveHullPocket} fields
    * accordingly. The bridge indices are required to call this method.
    * 
    * @param pocketToModify
    * @param concaveHullVertices
    * @return
    */
   public static boolean findDeepestVertexInPocket(ConcaveHullPocket pocketToModify, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      pocketToModify.clearDepthParameters();

      int startBridgeIndex = pocketToModify.getStartBridgeIndex();
      int endBridgeIndex = pocketToModify.getEndBridgeIndex();

      Point2DReadOnly startBridgeVertex = concaveHullVertices.get(startBridgeIndex);
      Point2DReadOnly endBridgeVertex = concaveHullVertices.get(endBridgeIndex);

      for (int index = next(startBridgeIndex, concaveHullVertices); index != endBridgeIndex; index = next(index, concaveHullVertices))
      {
         Point2DReadOnly vertex = concaveHullVertices.get(index);
         double depth = EuclidGeometryTools.distanceFromPoint2DToLine2D(vertex, startBridgeVertex, endBridgeVertex);

         if (depth > pocketToModify.getMaxDepth())
         {
            pocketToModify.setDeepestVertexIndex(index);
            pocketToModify.setDeepestVertex(vertex);
            pocketToModify.setMaxDepth(depth);
         }
      }

      return pocketToModify.getDeepestVertexIndex() >= 0;
   }

   public static ConcaveHullPocket findFirstConcaveHullPocketInefficient(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      ConvexPolygon2D convexHull = new ConvexPolygon2D(concaveHullVertices);

      // Find first common vertex between the two hulls. 
      int convexStartIndex = 0;
      int concaveStartIndex = -1;

      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point2DReadOnly currentConvexVertex = convexHull.getVertex(i);

         for (int j = 0; j < concaveHullVertices.size(); j++)
         {
            Point2DReadOnly currentConcaveVertex = concaveHullVertices.get(j);

            if (currentConcaveVertex.epsilonEquals(currentConvexVertex, 1.0e-7))
            {
               concaveStartIndex = j;
               break;
            }
         }

         if (convexStartIndex != -1)
            break;
      }

      if (convexStartIndex == -1 || concaveStartIndex == -1)
         return null;

      // Find the first index at which a bridge starts
      int startBridgeConcaveIndex = -1;
      int startBridgeConvexIndex = -1;

      for (int indexOffset = 1; indexOffset < concaveHullVertices.size(); indexOffset++)
      {
         int currentConvexIndex = (convexStartIndex + indexOffset) % convexHull.getNumberOfVertices();
         int currentConcaveIndex = (concaveStartIndex + indexOffset) % concaveHullVertices.size();
         Point2DReadOnly currentConvexVertex = convexHull.getVertex(currentConvexIndex);
         Point2DReadOnly currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

         if (!currentConvexVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
         {
            startBridgeConvexIndex = currentConvexIndex - 1;
            if (startBridgeConvexIndex == -1)
               startBridgeConvexIndex = convexHull.getNumberOfVertices() - 1;
            startBridgeConcaveIndex = currentConcaveIndex - 1;
            if (startBridgeConcaveIndex == -1)
               startBridgeConcaveIndex = concaveHullVertices.size() - 1;
            break;
         }
      }

      if (startBridgeConvexIndex == -1 || startBridgeConcaveIndex == -1)
         return null;

      // Find the deepest vertex in the pocket
      int endBridgeConcaveIndex = -1;

      Point2DReadOnly firstBridgeVertex = convexHull.getVertex(startBridgeConvexIndex);
      Point2DReadOnly secondBridgeVertex = convexHull.getNextVertex(startBridgeConvexIndex);
      LineSegment2D bridgeSegment = new LineSegment2D(firstBridgeVertex, secondBridgeVertex);

      int currentConcaveIndex = (startBridgeConcaveIndex + 1) % concaveHullVertices.size();
      Point2DReadOnly currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

      int deepestPocketVertexIndex = -1;
      double pocketMaxDepth = 0.0;

      while (!secondBridgeVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
      {
         double currentDepth = bridgeSegment.distance(currentConcaveVertex);

         if (currentDepth > pocketMaxDepth)
         {
            deepestPocketVertexIndex = currentConcaveIndex;
            pocketMaxDepth = currentDepth;
         }

         currentConcaveIndex = (currentConcaveIndex + 1) % concaveHullVertices.size();
         currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);
      }

      endBridgeConcaveIndex = currentConcaveIndex;

      ConcaveHullPocket pocket = new ConcaveHullPocket();
      pocket.setBridgeIndices(startBridgeConcaveIndex, endBridgeConcaveIndex);
      pocket.setBridgeVertices(firstBridgeVertex, secondBridgeVertex);
      pocket.setMaxDepth(pocketMaxDepth);
      pocket.setDeepestVertex(concaveHullVertices.get(deepestPocketVertexIndex));
      pocket.setDeepestVertexIndex(deepestPocketVertexIndex);

      return pocket;
   }

   public static boolean isConvexAtVertex(int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
      Point2DReadOnly previousVertex = concaveHullVertices.get(previous(vertexIndex, concaveHullVertices));
      Point2DReadOnly nextVertex = concaveHullVertices.get(next(vertexIndex, concaveHullVertices));

      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(vertex, previousVertex, nextVertex);
   }

   public static boolean isAlmostConvexAtVertex(int vertexIndex, double angleTolerance, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
      Point2DReadOnly previousVertex = getPrevious(vertexIndex, concaveHullVertices);
      Point2DReadOnly nextVertex = getNext(vertexIndex, concaveHullVertices);

      return getAngleABC(nextVertex, previousVertex, vertex) > -angleTolerance;
   }

   public static double getAngleABC(Point2DReadOnly nextVertex, Point2DReadOnly previousVertex, Point2DReadOnly vertex)
   {
      double bax = previousVertex.getX() - nextVertex.getX();
      double bay = previousVertex.getY() - nextVertex.getY();

      double bcx = previousVertex.getX() - vertex.getX();
      double bcy = previousVertex.getY() - vertex.getY();

      return EuclidGeometryTools.angleFromFirstToSecondVector2D(bax, bay, bcx, bcy);
   }

   public static double getAngleFromPreviousEdgeToNextEdge(int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
      Point2DReadOnly previousVertex = getPrevious(vertexIndex, concaveHullVertices);
      Point2DReadOnly nextVertex = getNext(vertexIndex, concaveHullVertices);

      double previousEdgeX = vertex.getX() - previousVertex.getX();
      double previousEdgeY = vertex.getY() - previousVertex.getY();
      double nextEdgeX = nextVertex.getX() - vertex.getX();
      double nextEdgeY = nextVertex.getY() - vertex.getY();
      return EuclidGeometryTools.angleFromFirstToSecondVector2D(previousEdgeX, previousEdgeY, nextEdgeX, nextEdgeY);
   }

   public static boolean isHullConvex(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      if (concaveHullVertices.size() <= 3)
         return true;

      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         if (!isConvexAtVertex(i, concaveHullVertices))
            return false;
      }

      return true;
   }

   /**
    * Find the closest edge to a vertex that is visible from the inside of the polygon. In other
    * words, the line that goes from the given vertex to the computed closestPoint is entirely
    * inside the polygon.
    * 
    * @param vertexIndex the close edge to this vertex.
    * @param deadIndexRegion the search algorithm starts at
    *           {@code vertexIndex + deadIndexRegion + 1} and ends at
    *           {@code vertexIndex - deadIndexRegion - 1}
    * @param concaveHullVertices the vertices of the concave hull on which the search is to be
    *           performed.
    * @param closestPointToPack coordinates of the closest point found.
    * @return the index of the closest edge first vertex.
    */
   public static int findInnerClosestEdgeToVertex(int vertexIndex, int deadIndexRegion, List<? extends Point2DReadOnly> concaveHullVertices,
                                                  Point2DBasics closestPointToPack)
   {
      int startSearchIndex = next(vertexIndex, concaveHullVertices);
      int endSearchIndex = previous(vertexIndex, concaveHullVertices);
      return findInnerClosestEdgeToVertex(vertexIndex, startSearchIndex, endSearchIndex, concaveHullVertices, closestPointToPack);
   }

   public static int findInnerClosestEdgeToVertex(int vertexIndex, int startSearchIndex, int endSearchIndex,
                                                  List<? extends Point2DReadOnly> concaveHullVertices, Point2DBasics closestPointToPack)
   {
      int closestEdgeFirstIndex = -1;
      double distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;

      vertexIndex %= concaveHullVertices.size();
      Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);

      LineSegment2D edge = new LineSegment2D();
      Point2D candidateClosestPoint = new Point2D();

      // The loop skips the edges to which the given vertex belongs.
      for (int candidateIndex = startSearchIndex; candidateIndex != endSearchIndex; candidateIndex = next(candidateIndex, concaveHullVertices))
      {
         Point2DReadOnly edgeFirstVertex = concaveHullVertices.get(candidateIndex);
         Point2DReadOnly edgeSecondVertex = getNext(candidateIndex, concaveHullVertices);

         edge.set(edgeFirstVertex, edgeSecondVertex);
         edge.orthogonalProjection(vertex, candidateClosestPoint);

         double distanceSquared = candidateClosestPoint.distanceSquared(vertex);

         // We have a new point that is closer than the previous.
         if (distanceSquared < distanceSquaredToClosestEdge)
         {
            // Before considering it as the new closest point, we need to check that the line goes from the given vertex to the new candidate is fully inside of the polygon.
            boolean isLineInsidePolygon = true;

            int startCheckIndex = next(startSearchIndex, concaveHullVertices);
            int endCheckIndex = next(candidateIndex, concaveHullVertices);

            // The line is inside the polygon if all the vertices in ]vertexIndex; candidateIndex[ are on the left side of the line (vertex, candidateClosestPoint)
            for (int index = startCheckIndex; index != endCheckIndex && isLineInsidePolygon; index = next(index, concaveHullVertices))
               isLineInsidePolygon = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(concaveHullVertices.get(index), vertex, candidateClosestPoint);

            if (isLineInsidePolygon)
            { // The line is inside, the candidate is the new closest point.
               distanceSquaredToClosestEdge = distanceSquared;
               closestEdgeFirstIndex = candidateIndex;
               closestPointToPack.set(candidateClosestPoint);
            }
         }
      }

      return closestEdgeFirstIndex;
   }

   public static int findClosestIntersectionWithRay(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, int startSearchIndex, int endSearchIndex,
                                                    List<? extends Point2DReadOnly> concaveHullVertices, Point2DBasics intersectionToPack)
   {
      double minDistanceSquared = Double.POSITIVE_INFINITY;
      int closestEdgeFirstVertexIndex = -1;
      intersectionToPack.set(Double.NaN, Double.NaN);

      Vector2D rayOriginToCandidate = new Vector2D();
      Line2D rayLine = new Line2D(rayOrigin, rayDirection);
      LineSegment2D edge = new LineSegment2D();

      for (int currentIndex = startSearchIndex; currentIndex != endSearchIndex; currentIndex = next(currentIndex, concaveHullVertices))
      {
         int nextIndex = next(currentIndex, concaveHullVertices);

         Point2DReadOnly current = concaveHullVertices.get(currentIndex);
         Point2DReadOnly next = concaveHullVertices.get(nextIndex);

         edge.set(current, next);
         Point2D intersection = edge.intersectionWith(rayLine);

         if (intersection != null)
         {
            rayOriginToCandidate.sub(intersection, rayOrigin);
            if (rayOriginToCandidate.dot(rayDirection) < 0.0)
               continue;

            double distanceSquared = intersection.distanceSquared(rayOrigin);
            if (distanceSquared < minDistanceSquared)
            {
               minDistanceSquared = distanceSquared;
               intersectionToPack.set(intersection);
               closestEdgeFirstVertexIndex = currentIndex;
            }
         }
      }
      return closestEdgeFirstVertexIndex;
   }

   public static String vertexListToString(List<? extends Point2DReadOnly> vertexList)
   {
      String ret = "";
      for (int i = 0; i < vertexList.size(); i++)
      {
         Point2DReadOnly vertex = vertexList.get(i);
         ret += vertex.getX() + ", " + vertex.getY();
         if (i < vertexList.size() - 1)
            ret += "\n";
      }
      return ret;
   }

   public static void exportVertexListToFile(List<? extends Point2DReadOnly> vertexList, String fileName)
   {
      try
      {
         File file = new File(fileName);
         if (!file.exists())
            file.createNewFile();
         FileWriter fileWriter = new FileWriter(file);
         fileWriter.write(vertexListToString(vertexList));
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
