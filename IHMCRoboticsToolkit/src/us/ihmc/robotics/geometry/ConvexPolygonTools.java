package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class ConvexPolygonTools
{
   private static final boolean DEBUG = false;

   public static class EmptyPolygonException extends RuntimeException
   {
      private static final long serialVersionUID = -323833885395952453L;

      public EmptyPolygonException(String description)
      {
         super(description);
      }
   }

   public static class OutdatedPolygonException extends RuntimeException
   {
      private static final long serialVersionUID = -5043468839061602341L;

      public OutdatedPolygonException(String description)
      {
         super(description);
      }
   }

   /**
    * Assumes that the polygons are disjoint.
    * Find the vertex indices corresponding to the end points of the two connecting edges.
    * @param polygon1 the first polygon
    * @param polygon2 the second polygon
    * @param verticesIndices in[2][2] contains the indexes of the connecting edges end points.
    * The row index refers to which polygon the vertex index belongs to, whereas the column index refers to which connecting edge the vertex index belongs to.
    * For example, vertexIndexes[0][1] is the index of the vertex of the first polygon, also end point the second connecting edge.
    * @return success (false = failed, true = succeeded)
    */
   public static boolean findConnectingEdgesVerticesIndexes(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, int[][] verticesIndices)
   {
      boolean success = false;

      if (polygon1.isEmpty() || polygon2.isEmpty())
         return false;

      if (polygon1.hasExactlyOneVertex() && polygon2.hasExactlyOneVertex())
      {
         verticesIndices[0][0] = 0;
         verticesIndices[0][1] = 0;
         verticesIndices[1][0] = 0;
         verticesIndices[1][1] = 0;
         return true;
      }

      if (polygon1.hasExactlyOneVertex())
      {
         verticesIndices[0][0] = 0;
         verticesIndices[0][1] = 0;
         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon1.getVertex(0), verticesIndices[1], polygon2);
         return success;
      }

      if (polygon2.hasExactlyOneVertex())
      {
         verticesIndices[1][0] = 0;
         verticesIndices[1][1] = 0;
         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon2.getVertex(0), verticesIndices[0], polygon1);
         return success;
      }

      // First pick a random vertex from polygon1
      Point2DReadOnly vertex = polygon1.getVertex(0);

      int[] lineOfSight1 = new int[2];
      int[] lineOfSight2 = new int[2];
      int L1, R1, L2, R2;

      // Then find its two line of sight points on polygon 2:
      success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(vertex, lineOfSight1, polygon2);
      if (!success)
         return false;

      L2 = lineOfSight1[0];
      R2 = lineOfSight1[1];

      // Then find the line of sight vertices on polygon 1:
      success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon2.getVertex(R2), lineOfSight1, polygon1);
      if (!success)
         return false;
      success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon2.getVertex(L2), lineOfSight2, polygon1);
      if (!success)
         return false;

      L1 = lineOfSight1[0];
      R1 = lineOfSight2[1];

      // Find the line of sight vertices back and forth between the two polygons until they are constant between two iterations.
      boolean done = false;
      while (!done)
      {
         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon1.getVertex(L1), lineOfSight1, polygon2);
         if (!success)
            return false;
         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon1.getVertex(R1), lineOfSight2, polygon2);
         if (!success)
            return false;

         if ((L2 == lineOfSight2[0]) && (R2 == lineOfSight1[1]))
         {
            done = true;
            break;
         }

         L2 = lineOfSight2[0];
         R2 = lineOfSight1[1];

         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon2.getVertex(L2), lineOfSight1, polygon1);
         if (!success)
            return false;
         success = ConvexPolygon2dCalculator.getLineOfSightVertexIndices(polygon2.getVertex(R2), lineOfSight2, polygon1);
         if (!success)
            return false;

         if ((L1 == lineOfSight2[0]) && (R1 == lineOfSight1[1]))
         {
            done = true;
            break;
         }

         L1 = lineOfSight2[0];
         R1 = lineOfSight1[1];
      }

      verticesIndices[0][0] = R1;
      verticesIndices[0][1] = L1;
      verticesIndices[1][0] = L2;
      verticesIndices[1][1] = R2;
      return true;
   }

   /**
    * Efficiently combines two Disjoint Polygons. Returns false if not disjoint.
    *
    * @param polygon1 ConvexPolygon2d
    * @param polygon2 ConvexPolygon2d
    * @param combinedPolygonToPack ConvexPolygon2d polygon in which we put the convex hull containing polygon1 and polygon2.
    * @param connectingEdge1ToPack LineSegment2d first connecting edge between polygon1 and polygon2.
    * @param connectingEdge2Topack LineSegment2d second connecting edge between polygon1 and polygon2.
    * @return true if succeeded, false if failed
    */
   public static boolean combineDisjointPolygons(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, ConvexPolygon2d combinedPolygonToPack,
         LineSegment2d connectingEdge1ToPack, LineSegment2d connectingEdge2Topack)
   {

      int[][] verticesIndices = new int[2][2];
      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, verticesIndices);
      if (!success)
         return false;

      combinedPolygonToPack.clear();
      polygon1.addVerticesInClockwiseOrderInPolygon(verticesIndices[0][1], verticesIndices[0][0], combinedPolygonToPack);
      polygon2.addVerticesInClockwiseOrderInPolygon(verticesIndices[1][0], verticesIndices[1][1], combinedPolygonToPack);
      combinedPolygonToPack.update();

      getConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);

      return true;
   }

   public static boolean findConnectingEdges(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, LineSegment2d connectingEdge1ToPack,
         LineSegment2d connectingEdge2Topack)
   {
      int[][] verticesIndices = new int[2][2];
      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, verticesIndices);
      if (!success)
         return false;

      getConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);
      return true;
   }

   private static void getConnectingEdges(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, LineSegment2d connectingEdge1ToPack,
         LineSegment2d connectingEdge2Topack, int[][] verticesIndices)
   {
      connectingEdge1ToPack.set(polygon1.getVertex(verticesIndices[0][0]), polygon2.getVertex(verticesIndices[1][0]));
      connectingEdge2Topack.set(polygon2.getVertex(verticesIndices[1][1]), polygon1.getVertex(verticesIndices[0][1]));
   }

   /**
    * Efficiently combines two Disjoint Polygons. Returns null if not disjoint.
    * Note: Generates garbage!
    *
    * @param polygon1 ConvexPolygon2d
    * @param polygon2 ConvexPolygon2d
    * @return ConvexPolygon2dAndConnectingEdges
    */
   public static ConvexPolygon2dAndConnectingEdges combineDisjointPolygons(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      ConvexPolygon2d combinedPolygon = new ConvexPolygon2d();
      LineSegment2d connectingEdge1 = new LineSegment2d();
      LineSegment2d connectingEdge2 = new LineSegment2d();
      boolean success = combineDisjointPolygons(polygon1, polygon2, combinedPolygon, connectingEdge1, connectingEdge2);
      if (!success)
         return null;

      return new ConvexPolygon2dAndConnectingEdges(combinedPolygon, connectingEdge1, connectingEdge2);
   }

   /**
    * Computes the intersection of two convex polygons.
    * For references see:
    * http://www.iro.umontreal.ca/~plante/compGeom/algorithm.html
    * Returns null if the polygons do not intersect
    * Returns the inside polygon if the two polygons are inside one another.
    *
    * @param polygonP ConvexPolygon2d
    * @param polygonQ ConvexPolygon2d
    * @return ConvexPolygon2d Intersection of polygonP and polygonQ
    */
   public static boolean computeIntersectionOfPolygons(ConvexPolygon2d polygonP, ConvexPolygon2d polygonQ, ConvexPolygon2d intersectingPolygonToPack)
   {
      // return false if either polygon null
      if (polygonP == null || polygonP.isEmpty())
         return false;
      if (polygonQ == null || polygonQ.isEmpty())
         return false;

      if (polygonP.hasExactlyTwoVertices() && polygonQ.hasAtLeastTwoVertices())
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonP, polygonQ,
               intersectingPolygonToPack);
      }

      else if (polygonQ.hasExactlyTwoVertices())
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonQ, polygonP,
               intersectingPolygonToPack);
      }

      if (polygonP.hasExactlyOneVertex() && polygonQ.hasAtLeastOneVertex())
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonP, polygonQ, intersectingPolygonToPack);
      }

      else if (polygonQ.hasExactlyOneVertex())
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonQ, polygonP, intersectingPolygonToPack);
      }

      // Find left most point on polygon1
      int currentPPolygonPointIndex = polygonP.getMinXIndex();
      int initialPolygonPIndex = polygonP.getMinXIndex();
      Point2DReadOnly currentPolygonPPoint = polygonP.getVertex(currentPPolygonPointIndex);

      // Find left most point on polygon2
      int currentQPolygonPointIndex = polygonQ.getMinXIndex();
      int initialPolygonQIndex = polygonQ.getMinXIndex();
      Point2DReadOnly currentPolygonQPoint = polygonQ.getVertex(currentQPolygonPointIndex);

      // At each of those two vertices, place a vertical line passing through it. Associate that line to the polygon to which the vertex belongs.
      Vector2D caliperForPolygonP = new Vector2D(0.0, 1.0);
      Vector2D caliperForPolygonQ = new Vector2D(0.0, 1.0);

      // determine which side polygon2's caliper line is on relative to polygon1's caliper line
      Point2D lineStart = new Point2D(currentPolygonPPoint);
      Point2D lineEnd = new Point2D(currentPolygonPPoint);
      lineEnd.add(caliperForPolygonP);
      boolean isOnLeft = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(currentPolygonQPoint, lineStart, lineEnd);
      boolean wasOnLeft = isOnLeft;

      //    System.out.println("wasOnLeft = " + wasOnLeft);

      boolean gotAroundPOnce = false;
      boolean gotAroundQOnce = false;
      boolean DONE = false;

      int bridgeCount = 0;
      ArrayList<Integer> bridgeIndicesP = new ArrayList<Integer>();
      ArrayList<Integer> bridgeIndicesQ = new ArrayList<Integer>();
      ArrayList<Boolean> bridgeWasOnLeft = new ArrayList<Boolean>();

      do
      {
         if (gotAroundPOnce && gotAroundQOnce)
         {
            DONE = true;
         }

         // Rotate these two lines (called calipers) by the smallest angle between a caliper and the segment following the vertex it passes
         // through (in clockwise order). The rotation is done about the vertex through which the line passes on the associated polygon.
         // If the line passes through more than one vertex of the assciated polygon, the farthest (in clockwise order) is taken.

         // find angle from current to next point for polygon1
         Vector2D vectorToNextPointOnPolygonP = new Vector2D(polygonP.getNextVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.sub(polygonP.getVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.normalize();

         // +++JEP: Don't actually compute the angle! Just look at the dot products!
         //       double angleToNextPointOnPolygonP = caliperForPolygonP.angle(vectorToNextPointOnPolygonP); //Math.acos(vectorToNextPointOnPolygon1.getY());
         double dotProductToNextPointOnPolygonP = caliperForPolygonP.dot(vectorToNextPointOnPolygonP); // Math.acos(vectorToNextPointOnPolygon1.getY());

         // find angle from current to next point for polygon2
         Vector2D vectorToNextPointOnPolygonQ = new Vector2D(polygonQ.getNextVertex(currentQPolygonPointIndex));
         vectorToNextPointOnPolygonQ.sub(polygonQ.getVertex(currentQPolygonPointIndex));
         vectorToNextPointOnPolygonQ.normalize();

         //       double angleToNextPointOnPolygonQ = caliperForPolygonQ.angle(vectorToNextPointOnPolygonQ); //Math.acos(vectorToNextPointOnPolygon2.getY());
         double dotProductToNextPointOnPolygonQ = caliperForPolygonQ.dot(vectorToNextPointOnPolygonQ); // Math.acos(vectorToNextPointOnPolygon2.getY());

         // determine the smallest angle and rotate both calipers by this amount
         boolean moveCaliperP = false;
         boolean moveCaliperQ = false;

         //       if (angleToNextPointOnPolygonP == angleToNextPointOnPolygonQ)
         if (dotProductToNextPointOnPolygonP == dotProductToNextPointOnPolygonQ)
         {
            caliperForPolygonP.set(vectorToNextPointOnPolygonP);
            caliperForPolygonQ.set(caliperForPolygonP);

            moveCaliperP = true;
            moveCaliperQ = true;
         }

         //       else if (angleToNextPointOnPolygonP < angleToNextPointOnPolygonQ)
         else if (dotProductToNextPointOnPolygonP > dotProductToNextPointOnPolygonQ)
         {
            caliperForPolygonP.set(vectorToNextPointOnPolygonP);
            caliperForPolygonQ.set(caliperForPolygonP);
            moveCaliperP = true;
         }
         else
         {
            caliperForPolygonQ.set(vectorToNextPointOnPolygonQ);
            caliperForPolygonP.set(caliperForPolygonQ);

            moveCaliperQ = true;
         }

         // Whenever the order of the two calipers change, a pocket has been found. To detect this, a direction is associated with one of
         // the lines (for example the green one, associated to P). Then all points of the red line (associated to Q) are either to the
         // left or to the right of the green line. When a rotation makes them change from one side to the other of the green line, then
         // the order of the two lines has changed.

         // detemine which side polygon Q's caliper line is on relative to polygon P's caliper lline
         lineStart = new Point2D(currentPolygonPPoint);
         lineEnd = new Point2D(currentPolygonPPoint);
         lineEnd.add(caliperForPolygonP);
         isOnLeft = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(currentPolygonQPoint, lineStart, lineEnd);

         //       System.out.println("new isOnLeft = " + isOnLeft);

         if (wasOnLeft != isOnLeft)
         {
            // find all of the bridges
            //          System.out.println("Found bridge. wasOnLeft = " + wasOnLeft + ", isOnLeft = " + isOnLeft);

            // Some weird fence post thing here. Sometime you want to consider the start ones at the end, sometimes you don't.
            // So that's why DONE is computed at the top and checked on the bottom...

            boolean addThisOne = true;
            if ((DONE) && (!bridgeIndicesP.isEmpty()))
            {
               if ((bridgeIndicesP.get(0) == currentPPolygonPointIndex) && (bridgeIndicesQ.get(0) == currentQPolygonPointIndex))
               {
                  addThisOne = false;
               }
            }

            if (addThisOne)
            {
               bridgeIndicesP.add(currentPPolygonPointIndex);
               bridgeIndicesQ.add(currentQPolygonPointIndex);
               bridgeWasOnLeft.add(wasOnLeft);

               // bridgeIndices1[bridgeCount] = currentPPolygonPointIndex;
               // bridgeIndices2[bridgeCount] = currentQPolygonPointIndex;
               // bridgeWasOnLeft[bridgeCount] = wasOnLeft;

               bridgeCount++;

               // update current caliper relationship
               wasOnLeft = isOnLeft;
            }
         }

         // The algorithm terminates once it has gone around both polygons
         if (moveCaliperP)
         {
            currentPPolygonPointIndex = polygonP.getNextVertexIndex(currentPPolygonPointIndex);
            currentPolygonPPoint = polygonP.getVertex(currentPPolygonPointIndex);

            if (currentPPolygonPointIndex == (initialPolygonPIndex) % polygonP.getNumberOfVertices())
            {
               gotAroundPOnce = true;
            }
         }

         if (moveCaliperQ)
         {
            currentQPolygonPointIndex = polygonQ.getNextVertexIndex(currentQPolygonPointIndex);
            currentPolygonQPoint = polygonQ.getVertex(currentQPolygonPointIndex);

            if (currentQPolygonPointIndex == (initialPolygonQIndex) % polygonQ.getNumberOfVertices())
            {
               gotAroundQOnce = true;
            }
         }
      }
      while (!DONE);

      // If no bridges, then check if the polygons are contained in each other.
      if (bridgeCount == 0)
      {
         // check to see if a polygons is contained in another.
         if (ConvexPolygon2dCalculator.isPointInside(polygonQ.getVertex(0), polygonP))
         {
            intersectingPolygonToPack.setAndUpdate(polygonQ);
         }

         if (ConvexPolygon2dCalculator.isPointInside(polygonP.getVertex(0), polygonQ))
         {
            intersectingPolygonToPack.setAndUpdate(polygonP);
         }
      }
      else
      {
         boolean success = buildCommonPolygonFromBridges(bridgeIndicesP, bridgeIndicesQ, bridgeWasOnLeft, polygonP, polygonQ,
               intersectingPolygonToPack);
         if (!success)
         {
            intersectingPolygonToPack.clearAndUpdate();
            return false;
         }
      }

      // Merge the inner chains to determine intersection
      return true;
   }

   private static boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(ConvexPolygon2d polygonWithExactlyOneVertex,
         ConvexPolygon2d otherPolygon, ConvexPolygon2d intersectingPolygon)
   {
      if (otherPolygon.pointIsOnPerimeter(polygonWithExactlyOneVertex.getVertex(0)))
      {
         intersectingPolygon.setAndUpdate(polygonWithExactlyOneVertex);
         return false;
      }
      else
      {
         intersectingPolygon.clearAndUpdate();
         return false;
      }
   }

   private static boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(
         ConvexPolygon2d polygonWithExactlyTwoVertices, ConvexPolygon2d polygonWithAtLeastTwoVertices, ConvexPolygon2d intersectingPolygon)
   {
      LineSegment2d polygonWithTwoVerticesAsLineSegment = new LineSegment2d(polygonWithExactlyTwoVertices.getVertex(0),
            polygonWithExactlyTwoVertices.getVertex(1));
      Point2D[] intersection = polygonWithAtLeastTwoVertices.intersectionWith(polygonWithTwoVerticesAsLineSegment);

      if (intersection == null)
      {
         intersectingPolygon.clearAndUpdate();
         return false;
      }
      else
      {
         intersectingPolygon.setAndUpdate(intersection, intersection.length);
         return true;
      }
   }

   private static int[][] findCrossingIndices(boolean decrementP, int bridgeIndexForPolygonP, int bridgeIndexForPolygonQ, ConvexPolygon2d polygonP,
         ConvexPolygon2d polygonQ)
   {
      int incrementP = 1, incrementQ = 1;
      if (decrementP)
         incrementP = -1;
      else
         incrementQ = -1;

      int indexPStart = bridgeIndexForPolygonP;
      int indexQStart = bridgeIndexForPolygonQ;

      int indexPEnd = (indexPStart + incrementP) % polygonP.getNumberOfVertices();
      int indexQEnd = (indexQStart + incrementQ) % polygonQ.getNumberOfVertices();

      if (indexPEnd < 0)
         indexPEnd = polygonP.getNumberOfVertices() - 1;
      if (indexQEnd < 0)
         indexQEnd = polygonQ.getNumberOfVertices() - 1;

      Point2DReadOnly linePStart = polygonP.getVertex(indexPStart);
      Point2DReadOnly linePEnd = polygonP.getVertex(indexPEnd);

      Point2DReadOnly lineQStart = polygonQ.getVertex(indexQStart);
      Point2DReadOnly lineQEnd = polygonQ.getVertex(indexQEnd);

      int initialPPolygonStartIndex = indexPStart;
      int initialQPolygonStartIndex = indexQStart;

      boolean finished;
      do
      {
         finished = true;

         while (decrementP ^ EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(lineQEnd, linePStart, linePEnd))
         {
            lineQStart = lineQEnd;

            indexQStart = indexQEnd;
            indexQEnd = (indexQEnd + incrementQ) % polygonQ.getNumberOfVertices();
            if (indexQEnd < 0)
               indexQEnd = polygonQ.getNumberOfVertices() - 1;

            lineQEnd = polygonQ.getVertex(indexQEnd);
            finished = false;

            if (indexQStart == initialQPolygonStartIndex)
               return null; // No intersection. Prevent infinite loop.
         }

         while ((!decrementP) ^ EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(linePEnd, lineQStart, lineQEnd))
         {
            linePStart = linePEnd;

            indexPStart = indexPEnd;
            indexPEnd = (indexPEnd + incrementP) % polygonP.getNumberOfVertices();
            if (indexPEnd < 0)
               indexPEnd = polygonP.getNumberOfVertices() - 1;

            linePEnd = polygonP.getVertex(indexPEnd);
            finished = false;

            if (indexPStart == initialPPolygonStartIndex)
               return null; // No intersection. Prevent infinite loop.
         }

      }
      while (!finished);

      return new int[][] { { indexPStart, indexPEnd }, { indexQStart, indexQEnd } };
   }

   private static boolean constructPolygonForIntersection(ArrayList<Boolean> decrementP, int[][][] crossingIndices, ConvexPolygon2d polygonP,
         ConvexPolygon2d polygonQ, ConvexPolygon2d intersectingPolygonToPack)
   {
      int startIndexP1 = crossingIndices[0][0][0];
      int endIndexP1 = crossingIndices[0][0][1];

      int startIndexQ1 = crossingIndices[0][1][0];
      int endIndexQ1 = crossingIndices[0][1][1];

      // Want to do things in clockwise order so that making the new polygon is fast. Should only be one of two cases.
      // a) P1/Q2 start->end increments, P2/Q1 start->end decrement; OR
      // b) P2/Q1 start->end increments, P1/Q2 start->end decrement; OR

      boolean incrementPNext;
      if ((startIndexP1 + 1) % polygonP.getNumberOfVertices() == endIndexP1)
      {
         incrementPNext = true;
      }
      else if ((startIndexQ1 + 1) % polygonQ.getNumberOfVertices() == endIndexQ1)
      {
         incrementPNext = false;
      }
      else
      {
         throw new RuntimeException("Neither P1, nor P2 increment!!!");
      }

      intersectingPolygonToPack.clear();

      // Start at the first intersection. Add it. Then march to the next intersection. Then continue.
      for (int i = 0; i < crossingIndices.length; i++)
      {
         int startIndexP = crossingIndices[i][0][0];
         int endIndexP = crossingIndices[i][0][1];

         int startIndexQ = crossingIndices[i][1][0];
         int endIndexQ = crossingIndices[i][1][1];

         Point2DReadOnly startP = polygonP.getVertex(startIndexP);
         Point2DReadOnly endP = polygonP.getVertex(endIndexP);
         Point2DReadOnly startQ = polygonQ.getVertex(startIndexQ);
         Point2DReadOnly endQ = polygonQ.getVertex(endIndexQ);

         Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(startP, endP, startQ, endQ);
         if (intersection == null)
         {
            System.err.println("intersection is null in constructPolygonForIntersection!. startP = " + startP + ", endP = " + endP + ", startQ = " + startQ + ", endQ = " + endQ);
            System.err.println("startIndexP = " + startIndexP + ", endIndexP = " + endIndexP);
            System.err.println("Returning null polygon");

            intersectingPolygonToPack.clear();
            intersectingPolygonToPack.update();
            return false;
         }

         intersectingPolygonToPack.addVertex(intersection);

         if (incrementPNext)
         {
            int indexP = crossingIndices[i][0][1]; // endIndexP;
            int indexPNext = crossingIndices[(i + 1) % crossingIndices.length][0][0];

            //          System.out.println("indexP = " + indexP + ", indexPNext = " + indexPNext);

            while (indexP != indexPNext)
            {
               intersectingPolygonToPack.addVertex(polygonP.getVertex(indexP));
               indexP = polygonP.getNextVertexIndex(indexP);
            }
         }
         else
         {
            int indexQ = crossingIndices[i][1][1]; // endIndexQ;
            int indexQNext = crossingIndices[(i + 1) % crossingIndices.length][1][0];

            //          System.out.println("indexQ = " + indexQ + ", indexQNext = " + indexQNext);

            while (indexQ != indexQNext)
            {
               intersectingPolygonToPack.addVertex(polygonQ.getVertex(indexQ));
               indexQ = polygonQ.getNextVertexIndex(indexQ);
            }
         }

         incrementPNext = !incrementPNext;
      }

      intersectingPolygonToPack.update();

      if (intersectingPolygonToPack.isEmpty())
         return false;

      return true;
   }

   private static boolean buildCommonPolygonFromBridges(ArrayList<Integer> bridgeIndicesP, ArrayList<Integer> bridgeIndicesQ,
         ArrayList<Boolean> bridgeWasOnLeft, ConvexPolygon2d polygonP, ConvexPolygon2d polygonQ, ConvexPolygon2d intersectingPolygonToPack)
   {
      int[][][] crossingIndices = new int[bridgeIndicesP.size()][][];

      for (int i = 0; i < crossingIndices.length; i++)
      {
         // find intersection for bridge
         int bridgeIndexForPolygonP = bridgeIndicesP.get(i);
         int bridgeIndexForPolygonQ = bridgeIndicesQ.get(i);

         // for each bridge, compute the intersection points

         crossingIndices[i] = findCrossingIndices(bridgeWasOnLeft.get(i), bridgeIndexForPolygonP, bridgeIndexForPolygonQ, polygonP, polygonQ);

         if (crossingIndices[i] == null)
         {
            intersectingPolygonToPack.clearAndUpdate();
            return false; // No intersection.
         }
      }

      boolean success = constructPolygonForIntersection(bridgeWasOnLeft, crossingIndices, polygonP, polygonQ, intersectingPolygonToPack);
      return success;
   }



   public static ConvexPolygon2d shrinkInto(ConvexPolygon2d polygonP, Point2DReadOnly referencePointInP, ConvexPolygon2d polygonQ)
   {
      if (polygonQ.hasAtLeastOneVertex() && !polygonQ.hasAtLeastThreeVertices())
      {
         return new ConvexPolygon2d(polygonQ);
      }

      ArrayList<Line2d> rays = new ArrayList<Line2d>();

      Point2D referencePointInPCopy = new Point2D(referencePointInP);

      int leftMostIndexOnPolygonQ = polygonQ.getMinXIndex();
      Point2DReadOnly vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2DReadOnly nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

      int leftMostIndexOnPolygonP = polygonP.getMinXIndex();
      Point2DReadOnly vertexP = polygonP.getVertex(leftMostIndexOnPolygonP);
      int nextVertexPIndex = polygonP.getNextVertexIndex(leftMostIndexOnPolygonP);
      Point2DReadOnly nextVertexP = polygonP.getVertex(nextVertexPIndex);

      forEachPolygonQ: for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         Vector2D edgeOnQ = new Vector2D(nextVertexQ.getX() - vertexQ.getX(), nextVertexQ.getY() - vertexQ.getY());

         int j = 0;
         while (j < polygonP.getNumberOfVertices())
         {
            Vector2D edgeOnP = new Vector2D(nextVertexP.getX() - vertexP.getX(), nextVertexP.getY() - vertexP.getY());
            double crossProduct = edgeOnQ.getX() * edgeOnP.getY() - edgeOnP.getX() * edgeOnQ.getY();
            if (crossProduct <= 0.0)
            {
               referencePointInPCopy.setX(referencePointInP.getX() + vertexQ.getX() - vertexP.getX());
               referencePointInPCopy.setY(referencePointInP.getY() + vertexQ.getY() - vertexP.getY());
               Line2d ray = new Line2d(referencePointInPCopy, edgeOnQ);
               rays.add(ray);

               vertexQ = nextVertexQ;
               nextVertexQIndex = polygonQ.getNextVertexIndex(nextVertexQIndex);
               nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

               continue forEachPolygonQ;
            }
            else
            {
               j++;
               vertexP = nextVertexP;
               nextVertexPIndex = polygonP.getNextVertexIndex(nextVertexPIndex);
               nextVertexP = polygonP.getVertex(nextVertexPIndex);
            }
         }

         throw new RuntimeException("Should never get here!!");
      }

      ConvexPolygonConstructorFromInteriorOfRays convexPolygonConstructorFromInteriorOfRays = new ConvexPolygonConstructorFromInteriorOfRays();

      ConvexPolygon2d polygonToReturn = new ConvexPolygon2d();

      boolean foundPolygon = convexPolygonConstructorFromInteriorOfRays.constructFromInteriorOfRays(rays, polygonToReturn);
      if (foundPolygon) return polygonToReturn;
      return null;
   }

   // TODO move to convexPolygon2d currently is only called from linesegment
   public static Point2D[] intersection(LineSegment2d lineSegment, ConvexPolygon2d convexPolygon)
   {
      Point2D[] intersectingPoints = convexPolygon.intersectionWith(lineSegment);

      if (intersectingPoints == null)
         return null;

      Point2D[] ret = new Point2D[intersectingPoints.length];

      for (int i = 0; i < intersectingPoints.length; i++)
      {
         ret[i] = new Point2D(intersectingPoints[i]);
      }

      return ret;

      //    throw new RuntimeException("Not yet implemented");
   }

   public static boolean combineDisjointPolygons(FrameConvexPolygon2d polygon1, FrameConvexPolygon2d polygon2, FrameConvexPolygon2d combinedPolygonToPack,
         FrameLineSegment2d connectingEdge1ToPack, FrameLineSegment2d connectingEdge2ToPack)
   {
      combinedPolygonToPack.clear(polygon1.getReferenceFrame());
      combinedPolygonToPack.checkReferenceFrameMatch(connectingEdge1ToPack);
      combinedPolygonToPack.checkReferenceFrameMatch(connectingEdge2ToPack);

      boolean success = combineDisjointPolygons(polygon1.convexPolygon, polygon2.convexPolygon, combinedPolygonToPack.convexPolygon,
            connectingEdge1ToPack.lineSegment, connectingEdge2ToPack.lineSegment);

      if (!success)
         return false;

//      combinedPolygonToPack.updateFramePoints();
      combinedPolygonToPack.update();

      return true;
   }

   /**
    * generates garbage
    * @param polygon1
    * @param polygon2
    * @return
    */
   //this should throw away points that are inside of the other polygon
   public static FrameConvexPolygon2dAndConnectingEdges combineDisjointPolygons(FrameConvexPolygon2d polygon1, FrameConvexPolygon2d polygon2)
   {
      ReferenceFrame referenceFrame = polygon1.getReferenceFrame();
      polygon2.checkReferenceFrameMatch(referenceFrame);

      ConvexPolygon2dAndConnectingEdges polygonAndEdges = combineDisjointPolygons(polygon1.convexPolygon, polygon2.convexPolygon);

      if (polygonAndEdges == null)
         return null; // Return null if not disjoint

      FrameConvexPolygon2d frameConvexPolygon2d = new FrameConvexPolygon2d(referenceFrame, polygonAndEdges.getConvexPolygon2d());
      FrameLineSegment2d connectingEdge1 = new FrameLineSegment2d(referenceFrame, polygonAndEdges.getConnectingEdge1());
      FrameLineSegment2d connectingEdge2 = new FrameLineSegment2d(referenceFrame, polygonAndEdges.getConnectingEdge2());

      FrameConvexPolygon2dAndConnectingEdges ret = new FrameConvexPolygon2dAndConnectingEdges(polygon1, polygon2, frameConvexPolygon2d, connectingEdge1,
            connectingEdge2);

      return ret;
   }

   public static int cutPolygonWithLine(FrameLine2d cuttingLine, FrameConvexPolygon2d polygonToCut, RobotSide sideOfLineToCut)
   {
      cuttingLine.checkReferenceFrameMatch(polygonToCut);
      return cutPolygonWithLine(cuttingLine.getLine2d(), polygonToCut.getConvexPolygon2d(), sideOfLineToCut);
   }

   public static int cutPolygonWithLine(Line2d cuttingLine, ConvexPolygon2d polygonToCut, RobotSide sideOfLineToCut)
   {
      Point2D[] intersectionPoints = polygonToCut.intersectionWith(cuttingLine);

      if (intersectionPoints == null || intersectionPoints.length == 1)
      {
         return -1;
      }
      else
      {
         int numberOfVerticesRemoved = 0;
         int index = 0;
         while (index < polygonToCut.getNumberOfVertices())
         {
            Point2D vertex = polygonToCut.getVertexUnsafe(index);
            if (cuttingLine.isPointOnSideOfLine(vertex, sideOfLineToCut))
            {
               polygonToCut.removeVertex(index);
               numberOfVerticesRemoved++;
            }
            else
            {
               index++;
            }
         }
         polygonToCut.addVertices(intersectionPoints, intersectionPoints.length);
         polygonToCut.update();
         return numberOfVerticesRemoved;
      }
   }

   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It is conservative in
    * the sense, that the modified polygon will be contained in the original polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(ConvexPolygon2d polygon, int desiredVertices)
   {
      polygon.checkNonEmpty();

      if (desiredVertices == 0)
      {
         polygon.clear();
         polygon.update();
         return;
      }

      int vertices = polygon.getNumberOfVertices();
      while (vertices > desiredVertices)
      {
         int removeVertex = -1;
         double shortestEdgeLength = Double.POSITIVE_INFINITY;
         Point2DReadOnly lastVertex = polygon.getVertex(0);
         for (int i = 1; i < vertices+1; i++)
         {
            Point2DReadOnly nextVertex = null;
            if (i == vertices)
            {
               nextVertex = polygon.getVertex(0);
            }
            else
            {
               nextVertex = polygon.getVertex(i);
            }
            double edgeLength = lastVertex.distance(nextVertex);
            if (edgeLength < shortestEdgeLength)
            {
               shortestEdgeLength = edgeLength;
               removeVertex = i;
            }
            lastVertex = nextVertex;
         }

         int idx1 = -1;
         int idx2 = -1;
         if (removeVertex == vertices)
         {
            idx1 = vertices-1;
            idx2 = 0;
         }
         else
         {
            idx1 = removeVertex;
            idx2 = removeVertex-1;
         }

         Point2DReadOnly vertexA = polygon.getVertex(idx1);
         Point2DReadOnly vertexB = polygon.getVertex(idx2);
         double xNew = (vertexA.getX() + vertexB.getX()) / 2.0;
         double yNew = (vertexA.getY() + vertexB.getY()) / 2.0;

         polygon.removeVertex(idx1);
         polygon.removeVertex(idx2);
         polygon.addVertex(xNew, yNew);
         polygon.update();

         vertices = polygon.getNumberOfVertices();
      }

      while (vertices < desiredVertices)
      {
         int index = -1;
         double longestEdgeLength = Double.NEGATIVE_INFINITY;
         Point2DReadOnly lastVertex = polygon.getVertex(0);
         for (int i = 1; i < vertices+1; i++)
         {
            Point2DReadOnly nextVertex = null;
            if (i == vertices)
            {
               nextVertex = polygon.getVertex(0);
            }
            else
            {
               nextVertex = polygon.getVertex(i);
            }

            double edgeLength = lastVertex.distance(nextVertex);
            if (edgeLength > longestEdgeLength)
            {
               longestEdgeLength = edgeLength;
               index = i;
            }
            lastVertex = nextVertex;
         }

         int idx1 = -1;
         int idx2 = -1;
         if (index == vertices)
         {
            idx1 = vertices-1;
            idx2 = 0;
         }
         else
         {
            idx1 = index;
            idx2 = index-1;
         }

         Point2DReadOnly vertexA = polygon.getVertex(idx1);
         Point2DReadOnly vertexB = polygon.getVertex(idx2);
         double xNew = (vertexA.getX() + vertexB.getX()) / 2.0;
         double yNew = (vertexA.getY() + vertexB.getY()) / 2.0;

         polygon.scale(1.0 - 10E-10);
         polygon.addVertex(xNew, yNew);
         polygon.update();

         vertices = polygon.getNumberOfVertices();
      }
   }

   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It is conservative in
    * the sense, that the modified polygon will be contained in the original polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(FrameConvexPolygon2d polygon, int desiredVertices)
   {
      limitVerticesConservative(polygon.getConvexPolygon2d(), desiredVertices);
   }

   @Deprecated
   public static void movePointInsidePolygonAlongLine(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d line)
   {
      // Defaults to 2mm for desired capture to prevent some jerky behavior with VirtualToePoints.. // TODO: remove
      double amountToBeInside = 0.002;
      movePointInsidePolygonAlongLine(point, polygon, line, amountToBeInside);
   }

   // TODO move to polygon?
   @Deprecated
   public static void movePointInsidePolygonAlongLine(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d line, double amountToBeInside)
   {
      if (!polygon.isPointInside(point))
      {
         FramePoint2d[] intersections = polygon.intersectionWith(line);
         if (intersections != null)
         {
            FramePoint2d intersectionToUse;
   
            if (intersections.length == 2)
            {
               double distanceSquaredToIntersection0 = point.distanceSquared(intersections[0]);
               double distanceSquaredToIntersection1 = point.distanceSquared(intersections[1]);
   
               if (distanceSquaredToIntersection0 <= distanceSquaredToIntersection1)
                  intersectionToUse = intersections[0];
               else
                  intersectionToUse = intersections[1];
   
   
               point.setX(intersectionToUse.getX());
               point.setY(intersectionToUse.getY());
   
               // Move in a little along the line:
               FrameLineSegment2d guideLineSegment = new FrameLineSegment2d(intersections);
               FrameVector2d frameVector2d = new FrameVector2d();
               guideLineSegment.getFrameVector(frameVector2d);
               if (intersectionToUse == intersections[1])
                  frameVector2d.scale(-1.0);
               frameVector2d.normalize();
               frameVector2d.scale(amountToBeInside);
   
               point.setX(point.getX() + frameVector2d.getX());
               point.setY(point.getY() + frameVector2d.getY());
            }
            else
            {
               throw new RuntimeException("This is interesting, shouldn't get here.");
            }
         }
         else
         {
            point.set(polygon.getClosestVertexCopy(line));
         }
      }
   }

   public static void movePointInsidePolygonAlongVector(FramePoint2d pointToMove, FrameVector2d vector, FrameConvexPolygon2d polygon, double distanceToBeInside)
      {
         if (polygon.getNumberOfVertices() < 2)
         {
            return;
         }
   
         if (distanceToBeInside < 0.0)
            throw new RuntimeException("distanceToBeInside = " + distanceToBeInside);
   
         FrameLine2d line = new FrameLine2d(pointToMove, vector);
         FramePoint2d[] intersections = polygon.intersectionWith(line);
   
         if (intersections != null)
         {
            if ((intersections.length != 2) && (intersections.length != 1))
               throw new RuntimeException("intersections.length != 2 && intersections.length != 1. intersections.length = " + intersections.length);
   
            if (intersections.length == 1)
            {
               pointToMove.set(intersections[0]);
   
               return;
            }
   
            // make sure it's inside or on the edge of the polygon
            boolean insidePolygon = polygon.isPointInside(pointToMove);
            if (!insidePolygon)
            {
               double minDistance = Double.POSITIVE_INFINITY;
               FramePoint2d closestIntersection = null;
               for (int i = 0; i < intersections.length; i++)
               {
                  FramePoint2d intersection = intersections[i];
                  double distance = pointToMove.distance(intersection);
                  if (distance < minDistance)
                  {
                     minDistance = distance;
                     closestIntersection = intersection;
                  }
               }
   
               pointToMove.set(closestIntersection);
            }
   
            // make sure distance constraint is met; if infeasible, use midpoint of intersections
            double distanceBetweenIntersections = intersections[0].distance(intersections[1]);
            boolean constraintFeasible = distanceBetweenIntersections > 2.0 * distanceToBeInside;
   
            if (constraintFeasible)
            {
               for (int i = 0; i < intersections.length; i++)
               {
                  double distance = intersections[i].distance(pointToMove);
                  if (distance < distanceToBeInside)
                  {
                     int j = 1 - i;
                     vector.sub(intersections[j], intersections[i]);
                     vector.normalize();
                     vector.scale(distanceToBeInside);
                     pointToMove.set(intersections[i]);
                     pointToMove.add(vector);
                  }
               }
            }
            else
            {
               pointToMove.interpolate(intersections[0], intersections[1], 0.5);
            }
         }
         else
         {
            pointToMove.set(polygon.getClosestVertexCopy(line));
   
         }
   
   //    else
   //    {
   //       StringBuilder stringBuilder = new StringBuilder();
   //       stringBuilder.append("intersections == null\n");
   //       stringBuilder.append("pointToMove = " + pointToMove + "\n");
   //       stringBuilder.append("vector = " + vector + "\n");
   //       stringBuilder.append("polygon = " + polygon + "\n");
   //       stringBuilder.append("distanceToBeInside = " + distanceToBeInside);
   //
   //       throw new RuntimeException(stringBuilder.toString());
   //    }
      }

   public static void projectOntoPolygonAndCheckDistance(FramePoint2d point, FrameConvexPolygon2d polygon, double epsilon)
   {
      ReferenceFrame originalReferenceFrame = point.getReferenceFrame();
      point.changeFrame(polygon.getReferenceFrame());
      FramePoint2d originalPoint = new FramePoint2d(point);
      polygon.orthogonalProjection(point);
      double distance = originalPoint.distance(point);
      if (distance > epsilon)
         throw new RuntimeException("point outside polygon by " + distance);
      point.changeFrame(originalReferenceFrame);
   }

   /**
    * Finds the minimum distance between two convex polygons
    * Taken from http://cygnus-x1.cs.duke.edu/~edels/Papers/1985-J-02-ComputingExtremeDistances.pdf
    * @return Two points, one from each polygon, between which is the minimum distance between the two polygons
    */
   public static Point2DReadOnly[] computeMinimumDistancePoints(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, double epsilon)
   {
      // TODO Do something more clever than actually computing the intersection there!
      if (computeIntersectionOfPolygons(polygon1, polygon2, new ConvexPolygon2d()))
      {
         throw new RuntimeException("Cannot compute minimum distance between intersecting polygons.");
      }
   
      if ((polygon1.getNumberOfVertices() < 3) || (polygon2.getNumberOfVertices() < 3))
      {
         throw new RuntimeException("Polygon inputs are degenerate.");
      }
   
      int[] v1Tangents = findStartAndEndTangents(polygon2.getVertex(0), polygon1, epsilon);
      int[] v2Tangents = findStartAndEndTangents(polygon1.getVertex(0), polygon2, epsilon);
   
      int v1Start = v1Tangents[0];
      int v1End = v1Tangents[1];
      int v2Start = v2Tangents[0];
      int v2End = v2Tangents[1];
   
      int[] updatedIndices = binaryElimination(polygon1, polygon2, v1Start, v1End, v2Start, v2End, epsilon);
      v1Start = updatedIndices[0];
      v1End = updatedIndices[1];
      v2Start = updatedIndices[2];
      v2End = updatedIndices[3];
   
      return getClosestPointsFromRemainingEdgesAndVertices(polygon1, polygon2, v1Start, v1End, v2Start, v2End);
   }

   public static Point2DReadOnly[] computeMinimumDistancePoints(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      return computeMinimumDistancePoints(polygon1, polygon2, .01);
   }

   // TODO potentially implement [Chazelle and Dobkin] to get logarithmic running time for computeMinimumDistancePoints (though it would actually be log^2 in current
   // implementation, since binaryElimination, which has is O(log(n)) uses this method in each loop)

   /**
    * Finds the indices of the vertices of the polygon that form tangent lines to the polygon with the parameter point
    * @return The desired indices, ordered such that they form a range that includes all vertices visible from the parameter point; if there are more than two
    *          only returns the two necessary to specify this range
    */
   private static int[] findStartAndEndTangents(Point2DReadOnly point, ConvexPolygon2d polygon, double epsilon)
   {
      int tangentIndex1;
      int tangentIndex2;
   
      int vIndex = 0;
   
      while (!pointMakesTangentToPolygon(polygon, point, vIndex, epsilon))
      {
         vIndex++;
         vIndex %= polygon.getNumberOfVertices();
      }
   
      tangentIndex1 = vIndex;
      Vector2D tangent1 = new Vector2D(polygon.getVertex(tangentIndex1).getX() - point.getX(),polygon.getVertex(tangentIndex1).getY() - point.getY());
   
      vIndex++;
      vIndex %= polygon.getNumberOfVertices();
   
      while (!pointMakesTangentToPolygon(polygon, point, vIndex, epsilon))
      {
         vIndex++;
         vIndex %= polygon.getNumberOfVertices();
      }
   
      tangentIndex2 = vIndex;
      Vector2D tangent2 = new Vector2D(polygon.getVertex(tangentIndex2).getX() - point.getX(), polygon.getVertex(tangentIndex2).getY() - point.getY());
   
      if (GeometryTools.getAngleFromFirstToSecondVector(tangent1, tangent2) > 0)
      {
         return new int[] {tangentIndex1, tangentIndex2};
      }
   
      return new int[] {tangentIndex2, tangentIndex1};
   }

   /**
    * Uses the fact that if a line passes through a vertex of a convex polygon, the angles to the adjacent edges are going to be in opposite directions
    * @return Whether or not the line including the point and vertex is tangent to the polygon
    */
   private static boolean pointMakesTangentToPolygon(ConvexPolygon2d polygon, Point2DReadOnly point, int vertexIndex, double epsilon)
   {
      Point2DReadOnly vertex = polygon.getVertex(vertexIndex);
      Point2DReadOnly previous = polygon.getPreviousVertex(vertexIndex);
      Point2DReadOnly next = polygon.getNextVertex(vertexIndex);
   
      Vector2D base = new Vector2D(point.getX() - vertex.getX(), point.getY() - vertex.getY());
      Vector2D first = new Vector2D(previous.getX() - vertex.getX(), previous.getY() - vertex.getY());
      Vector2D second = new Vector2D(next.getX() - vertex.getX(), next.getY() - vertex.getY());
      double firstAngle = GeometryTools.getAngleFromFirstToSecondVector(base, first);
      double secondAngle = GeometryTools.getAngleFromFirstToSecondVector(base, second);
   
      if (firstAngle * secondAngle >= 0)
      {    // if both angles have the same sign, the line does not pass through the polygon
         return true;
      }
   
      if (MathTools.epsilonEquals(firstAngle, 0, epsilon) || MathTools.epsilonEquals(secondAngle, 0, epsilon))
      {    // if either angle is close to 0, assume floating point arithmetic error
         return true;
      }
   
      return false;
   }

   /**
    * Checks if index is within range; if low is greater than high, this implies a modularly cyclical range
    * @return True if the index is between low and high
    */
   private static boolean isInRange(int index, int low, int high)
   {
      if ((low <= index) && (index <= high))
      {
         return true;
      }
   
      if ((high < low) && ((index >= low) || (index <= high)))
      {
         return true;
      }
   
      return false;
   }

   /**
    * Eliminates vertices and return a range for each polygon, each of which comprises of at most two vertices
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryElimination(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, int v1Start, int v1End, int v2Start, int v2End, double epsilon)
   {
      Point2DReadOnly v1Median;
      Point2DReadOnly v2Median;
   
      int numberOfVertices1 = polygon1.getNumberOfVertices();
      int numberOfVertices2 = polygon2.getNumberOfVertices();
   
      while (((numberOfVertices1 + v1End - v1Start) % numberOfVertices1 + 1 > 2) || ((numberOfVertices2 + v2End - v2Start) % numberOfVertices2 + 1 > 2))
      {
         int v1MedianIndex = (v1Start <= v1End) ? (v1End + v1Start + 1) / 2 : ((v1End + v1Start + 1 + numberOfVertices1) / 2) % numberOfVertices1;
         int v2MedianIndex = (v2Start <= v2End) ? (v2End + v2Start) / 2 : ((v2End + v2Start + numberOfVertices2) / 2) % numberOfVertices2;
         v1Median = polygon1.getVertex(v1MedianIndex);
         v2Median = polygon2.getVertex(v2MedianIndex);
   
         Vector2D m = new Vector2D(v2Median.getX() - v1Median.getX(), v2Median.getY() - v1Median.getY());
         Vector2D mReversed = new Vector2D(v1Median.getX() - v2Median.getX(), v1Median.getY() - v2Median.getY());
   
         int edge1AStart = ((v1MedianIndex + numberOfVertices1 - 1) % numberOfVertices1);
         int edge1BEnd = (v1MedianIndex + 1) % numberOfVertices1;
         int edge2BStart = ((v2MedianIndex + numberOfVertices2 - 1) % numberOfVertices2);
         int edge2AEnd = (v2MedianIndex + 1) % numberOfVertices2;
         Vector2D edge1A = new Vector2D(polygon1.getVertex(edge1AStart).getX() - v1Median.getX(), polygon1.getVertex(edge1AStart).getY() - v1Median.getY());
         Vector2D edge1B = new Vector2D(polygon1.getVertex(edge1BEnd).getX() - v1Median.getX(), polygon1.getVertex(edge1BEnd).getY() - v1Median.getY());
         Vector2D edge2A = new Vector2D(polygon2.getVertex(edge2AEnd).getX() - v2Median.getX(), polygon2.getVertex(edge2AEnd).getY() - v2Median.getY());
         Vector2D edge2B = new Vector2D(polygon2.getVertex(edge2BStart).getX() - v2Median.getX(), polygon2.getVertex(edge2BStart).getY() - v2Median.getY());
   
         // see diagram 3.2 in [Edelsbrunner]
         double angle1A = GeometryTools.getAngleFromFirstToSecondVector(m, edge1A); // A' in diagram
         double angle1B = GeometryTools.getAngleFromFirstToSecondVector(edge1B, m); // A'' in diagram
         double angle2A = GeometryTools.getAngleFromFirstToSecondVector(edge2A, mReversed); // B' in diagram
         double angle2B = GeometryTools.getAngleFromFirstToSecondVector(mReversed, edge2B); // B'' in diagram
   
         int[] range1 = findStartAndEndTangents(v2Median, polygon1, epsilon);
         int[] range2 = findStartAndEndTangents(v1Median, polygon2, epsilon);
   
         angle1A = MathTools.epsilonEquals(angle1A, 0, .01) ? 0 : angle1A;
         angle1B = MathTools.epsilonEquals(angle1B, 0, .01) ? 0 : angle1B;
         angle2A = MathTools.epsilonEquals(angle2A, 0, .01) ? 0 : angle2A;
         angle2B = MathTools.epsilonEquals(angle2B, 0, .01) ? 0 : angle2B;
   
         angle1A += ((angle1A < 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle1B += ((angle1B < 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle2A += ((angle2A < 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;
         angle2B += ((angle2B < 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;
   
         angle1A += ((angle1A < 0) && (angle1B < 0) && (angle1A < angle1B)) ? 2 * Math.PI : 0;
         angle1B += ((angle1A < 0) && (angle1B < 0) && (angle1B < angle1A)) ? 2 * Math.PI : 0;
         angle2A += ((angle2A < 0) && (angle2B < 0) && (angle2A < angle2B)) ? 2 * Math.PI : 0;
         angle2B += ((angle2A < 0) && (angle2B < 0) && (angle2B < angle2A)) ? 2 * Math.PI : 0;
   
         int[] updatedIndices;
   
         if ((v1Start == v1End) || (v2Start == v2End))
         {
            updatedIndices = binaryEliminationCase1(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
         else if ((v1End - v1Start + numberOfVertices1) % numberOfVertices1 == 1)
         {
            updatedIndices = binaryEliminationCase2(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
         else if ((v2End - v2Start + numberOfVertices2) % numberOfVertices2 == 1)
         {
            updatedIndices = binaryEliminationCase2(angle2A, angle2B, angle1A, angle1B, v2End, v2MedianIndex, v2Start, v1End, v1MedianIndex, v1Start, polygon1, polygon2);
            v2End = updatedIndices[0];
            v2Start = updatedIndices[1];
            v1End = updatedIndices[2];
            v1Start = updatedIndices[3];
         }
         else
         {
            updatedIndices = binaryEliminationCase3(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
      }
   
      return new int[] { v1Start, v1End, v2Start, v2End };
   }

   /**
    * Binary elimination helper method called if one range has a size of exactly one
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase1(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End, ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      if (v1Start == v1End)
      {    // v1 contains only 1 viable vertex
         if (angle2A >= Math.PI / 2)
         {
            v2End = v2MedianIndex;
         }
   
         if (angle2B >= Math.PI / 2)
         {
            v2Start = v2MedianIndex;
         }
      }
      else if (v2Start == v2End)
      {
         if (angle1A >= Math.PI / 2)
         {
            v1Start = v1MedianIndex;
         }
   
         if (angle1B >= Math.PI / 2)
         {
            v1End = v1MedianIndex;
         }
      }
   
      return new int[] {v1Start, v1End, v2Start, v2End};
   }

   /**
    * Binary elimination helper method called if one range has a size of exactly two
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase2(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End, ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      if (angle1A > 0)
      {
         if (angle1A + angle2A >= Math.PI)
         {
            if (angle1A >= Math.PI / 2)
            {
               v1Start = v1End;
            }
   
            if (angle2A >= Math.PI / 2)
            {
               v2End = v2MedianIndex;
            }
         }
   
         if (angle2B >= Math.PI / 2)
         {
            v2Start = v2MedianIndex;
         }
   
         if ((angle1A < angle2B) && (angle2B < Math.PI / 2))
         {
            Point2D proj = EuclidGeometryTools.orthogonalProjectionOnLine2D(polygon2.getVertex(v2MedianIndex), polygon1.getVertex(v1Start), polygon1.getVertex(v1End));
            LineSegment2d p = new LineSegment2d(polygon1.getVertex(v1Start), polygon1.getVertex(v1End));
            if (p.isBetweenEndpoints(proj, 0))
            {
               v2Start = v2MedianIndex;
            }
            else
            {
               v1End = v1Start;
            }
         }
      }
      else
      {
         v1End = v1Start;
   
         if (angle2A >= Math.PI)
         {
            v2End = v2MedianIndex;
         }
   
         if (angle2B >= Math.PI)
         {
            v2Start = v2MedianIndex;
         }
      }
   
      return new int[] {v1Start, v1End, v2Start, v2End};
   }

   /**
    * Binary Elimination helper method called if both ranges have size greater than two
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase3(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End)
   {
      if ((angle1A > 0) && (angle1B > 0) && (angle2A > 0) && (angle2B > 0))
      {
         if (angle1A + angle2A > Math.PI)
         {
            if (angle1A >= Math.PI / 2)
            {
               v1Start = v1MedianIndex;
            }
   
            if (angle2A >= Math.PI / 2)
            {
               v2End = v2MedianIndex;
            }
         }
   
         if (angle1B + angle2B > Math.PI)
         {
            if (angle1B >= Math.PI / 2)
            {
               v1End = v1MedianIndex;
            }
   
            if (angle2B >= Math.PI / 2)
            {
               v2Start = v2MedianIndex;
            }
         }
      }
   
      if (angle1A <= 0)
      {
         v1End = v1MedianIndex;
      }
   
      if (angle1B <= 0)
      {
         v1Start = v1MedianIndex;
      }
   
      if (angle2A <= 0)
      {
         v2Start = v2MedianIndex;
      }
   
      if (angle2B <= 0)
      {
         v2End = v2MedianIndex;
      }
   
      return new int[] {v1Start, v1End, v2Start, v2End};
   }

   /**
    * Takes in two ranges each of which are of size at most two and returns the two points on each respective polygon which are closest to one another
    */
   private static Point2DReadOnly[] getClosestPointsFromRemainingEdgesAndVertices(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, int v1Start, int v1End, int v2Start, int v2End)
   {
      if ((v1Start == v1End) && (v2Start == v2End))
      {
         return new Point2DReadOnly[] {polygon1.getVertex(v1Start), polygon2.getVertex(v2Start)};
      }
   
      else if (v1Start == v1End)
      {
         return finalPhasePointAndEdge(polygon2.getVertex(v2Start), polygon2.getVertex(v2End), polygon1.getVertex(v1Start));
      }
   
      else if (v2Start == v2End)
      {
         Point2DReadOnly[] reverseOutput = finalPhasePointAndEdge(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start));
   
         return new Point2DReadOnly[] {reverseOutput[1], reverseOutput[0]};    // switch order of output so that points are returned in the order that their polygons were inputed
      }
   
      return finalPhaseTwoEdges(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start), polygon2.getVertex(v2End));
   }

   /**
    * Final phase helper method called if each range has size of exactly two
    * @return The two points on each respective polygon which are closest to one another
    */
   private static Point2DReadOnly[] finalPhaseTwoEdges(Point2DReadOnly edgePoint1A, Point2DReadOnly edgePoint1B, Point2DReadOnly edgePoint2A, Point2DReadOnly edgePoint2B)
   {
      LineSegment2d edge1 = new LineSegment2d(edgePoint1A, edgePoint1B);
      LineSegment2d edge2 = new LineSegment2d(edgePoint2A, edgePoint2B);
      Point2D proj1AOnto2 = EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint1A, edgePoint2A, edgePoint2B);
      Point2D proj1BOnto2 = EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint1B, edgePoint2A, edgePoint2B);
      Point2D proj2AOnto1 = EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint2A, edgePoint1A, edgePoint1B);
      Point2D proj2BOnto1 = EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint2B, edgePoint1A, edgePoint1B);
   
      Point2DReadOnly[][] possiblePointPairsWithProj = new Point2DReadOnly[4][2];
      Point2DReadOnly[][] possiblePointPairsWithoutProj = new Point2DReadOnly[4][2];
      double[] possibleDistancesWithProj = new double[4];
      double[] possibleDistancesWithoutProj = new double[4];
   
      possiblePointPairsWithProj[0] = edge2.isBetweenEndpoints(proj1AOnto2, 0) ? new Point2DReadOnly[] {edgePoint1A, proj1AOnto2} : null;
      possiblePointPairsWithProj[1] = edge2.isBetweenEndpoints(proj1BOnto2, 0) ? new Point2DReadOnly[] {edgePoint1B, proj1BOnto2} : null;
      possiblePointPairsWithProj[2] = edge1.isBetweenEndpoints(proj2AOnto1, 0) ? new Point2DReadOnly[] {proj2AOnto1, edgePoint2A} : null;
      possiblePointPairsWithProj[3] = edge1.isBetweenEndpoints(proj2BOnto1, 0) ? new Point2DReadOnly[] {proj2BOnto1, edgePoint2B} : null;
   
      possiblePointPairsWithoutProj[0] = new Point2DReadOnly[] {edgePoint1A, edgePoint2A};
      possiblePointPairsWithoutProj[1] = new Point2DReadOnly[] {edgePoint1A, edgePoint2B};
      possiblePointPairsWithoutProj[2] = new Point2DReadOnly[] {edgePoint1B, edgePoint2A};
      possiblePointPairsWithoutProj[3] = new Point2DReadOnly[] {edgePoint1B, edgePoint2B};
   
      for (int i = 0; i < 4; i++)
      {
         possibleDistancesWithProj[i] = (possiblePointPairsWithProj[i] == null)
                                        ? Double.MAX_VALUE : possiblePointPairsWithProj[i][0].distance(possiblePointPairsWithProj[i][1]);
         possibleDistancesWithoutProj[i] = possiblePointPairsWithoutProj[i][0].distance(possiblePointPairsWithoutProj[i][1]);
      }
   
      if (possibleDistancesWithProj[indexOfMin(possibleDistancesWithProj)] != Double.MAX_VALUE)
      {
         return possiblePointPairsWithProj[indexOfMin(possibleDistancesWithProj)];
      }
   
      return possiblePointPairsWithoutProj[indexOfMin(possibleDistancesWithoutProj)];
   }

   /**
    * @return Index of the minimum element in an array of doubles
    */
   private static int indexOfMin(double[] d)
   {
      if ((d == null) || (d.length == 0))
      {
         throw new RuntimeException("Cannot find minimum of empty or null array.");
      }
   
      int minIndex = 0;
      double minValue = d[minIndex];
      int searchIndex = 1;
      while (searchIndex < d.length)
      {
         if (d[searchIndex] < minValue)
         {
            minIndex = searchIndex;
            minValue = d[searchIndex];
         }
   
         searchIndex++;
      }
   
      return minIndex;
   }

   /**
    * Final phase helper method called if one range has a size of exactly one
    * @return The two points on each respective polygon which are closest to one another
    */
   private static Point2DReadOnly[] finalPhasePointAndEdge(Point2DReadOnly edgePoint1, Point2DReadOnly edgePoint2, Point2DReadOnly lonePoint)
   {
      Point2D proj = EuclidGeometryTools.orthogonalProjectionOnLine2D(lonePoint, edgePoint1, edgePoint2);
      LineSegment2d p = new LineSegment2d(edgePoint1, edgePoint2);
      if (p.isBetweenEndpoints(proj, 0))
      {
         return new Point2DReadOnly[] {lonePoint, proj};
      }
      else
      {
         return new Point2DReadOnly[] {lonePoint, (lonePoint.distance(edgePoint1) < lonePoint.distance(edgePoint2)) ? edgePoint1 : edgePoint2};
      }
   }

   /**
    * from http://softsurfer.com/Archive/algorithm_0111/algorithm_0111.htm#Pseudo-Code:%20Clip%20Segment-Polygon
    * Input: a 2D segment S from point P0 to point P1
    * a 2D convex polygon W with n vertices V0,...,Vn-1,Vn=V0
    */
   public static boolean doesSegmentIntersectConvexPolygon2D(Point2D P0, Point2D P1, ConvexPolygon2d convexPolygon2d)
   {
      // if segment is a single point
      if (P0.equals(P1))
      {
         return convexPolygon2d.isPointInside(P0);
      }
   
      // if either point is inside polygon
      if (convexPolygon2d.isPointInside(P0, .0001) || convexPolygon2d.isPointInside(P1, .0001))
         return true;
   
      // if either point touches the polygon
      if (convexPolygon2d.pointIsOnPerimeter(P0) || convexPolygon2d.pointIsOnPerimeter(P1))
         return true;
   
      return doesSegmentPassCompletelyThroughPolygon(P0, P1, convexPolygon2d);
   }

   private static boolean doesSegmentPassCompletelyThroughPolygon(Point2D P0, Point2D P1, ConvexPolygon2d convexPolygon2d)
   {
      // Initialize:
      double tE = 0.0;    // for the maximum entering segment parameter;
      double tL = 1.0;    // for the minimum leaving segment parameter;
   
      // segment direction vector
      Vector2D dS = new Vector2D(P1);
      dS.sub(P0);
   
      if (DEBUG)
      {
         System.out.println("dS = " + dS);
      }
   
      int numberOfVertices = convexPolygon2d.getNumberOfVertices();
      if (DEBUG)
      {
         System.out.println("ccwPoints = ");
   
         for (int i = 0; i < numberOfVertices; i++)
         {
            System.out.println(convexPolygon2d.getVertexCCW(i));
         }
      }
   
      for (int i = 0; i < numberOfVertices; i++)
      {
         // edge vertices
         Point2D V0 = new Point2D(convexPolygon2d.getVertexCCW(i));
         if (DEBUG)
         {
            System.out.println("V0 = " + V0);
         }
   
         Point2D V1 = new Point2D(convexPolygon2d.getNextVertexCCW(i));
         if (DEBUG)
         {
            System.out.println("V1 = " + V1);
         }
   
         // edge vector
         Vector2D V0toV1 = new Vector2D(V1);
         V0toV1.sub(V0);
   
         if (DEBUG)
         {
            System.out.println("V0toV1 = " + V0toV1);
         }
   
         // outward normal of the edge
         Vector2D ni = new Vector2D(V0toV1.getY(), -V0toV1.getX());
         if (DEBUG)
         {
            System.out.println("ni = " + ni);
         }
   
         Vector2D P0toVi = new Vector2D(P0);
         P0toVi.sub(V0);
   
         if (DEBUG)
         {
            System.out.println("P0toVi = " + P0toVi);
         }
   
         double N = -P0toVi.dot(ni);
         if (DEBUG)
         {
            System.out.println("N = " + N);
         }
   
         double D = dS.dot(ni);
         if (DEBUG)
         {
            System.out.println("D = " + D);
         }
   
         if (D == 0)
         {
            // S is parallel to the edge ei
   
            if (N < 0)
            {
               // then P0 is outside the edge ei
               return false;    // since S cannot intersect W;
            }
            else
            {
               // S cannot enter or leave W across edge ei
               // ignore edge ei and process the next edge
               continue;
            }
         }
   
         double t = N / D;
         if (DEBUG)
         {
            System.out.println("t = " + t);
         }
   
         if (D < 0)
         {
            // then segment S is entering W across edge ei
            tE = Math.max(tE, t);
   
            if (tE > tL)
            {
               // then segment S enters W after leaving
               return false;    // since S cannot intersect W
            }
         }
         else if (D > 0)
         {
            // then segment S is leaving W across edge ei
            tL = Math.min(tL, t);
   
            if (tL < tE)
            {
               // then segment S leaves W before entering
               return false;    // since S cannot intersect W
            }
         }
      }
   
      // Output: [Note: to get here, one must have tE <= tL]
      // there is a valid intersection of S with W
      // from the entering point: P(tE) = P0 + tE * dS
      // to the leaving point:    P(tL) = P0 + tL * dS
      return true;
   }
}
