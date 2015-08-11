package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.util.ArrayList;

public class ConvexPolygonTools
{

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
         success = polygon2.getLineOfSightVerticesIndices(polygon1.getVertex(0), verticesIndices[1]);
         return success;
      }

      if (polygon2.hasExactlyOneVertex())
      {
         verticesIndices[1][0] = 0;
         verticesIndices[1][1] = 0;
         success = polygon1.getLineOfSightVerticesIndices(polygon2.getVertex(0), verticesIndices[0]);
         return success;
      }

      // First pick a random vertex from polygon1
      Point2d vertex = polygon1.getVertex(0);

      int[] lineOfSight1 = new int[2];
      int[] lineOfSight2 = new int[2];
      int L1, R1, L2, R2;

      // Then find its two line of sight points on polygon 2:
      success = polygon2.getLineOfSightVerticesIndices(vertex, lineOfSight1);
      if (!success)
         return false;

      L2 = lineOfSight1[0];
      R2 = lineOfSight1[1];

      // Then find the line of sight vertices on polygon 1:
      success = polygon1.getLineOfSightVerticesIndices(polygon2.getVertex(R2), lineOfSight1);
      if (!success)
         return false;
      success = polygon1.getLineOfSightVerticesIndices(polygon2.getVertex(L2), lineOfSight2);
      if (!success)
         return false;

      L1 = lineOfSight1[0];
      R1 = lineOfSight2[1];

      // Find the line of sight vertices back and forth between the two polygons until they are constant between two iterations.
      boolean done = false;
      while (!done)
      {
         success = polygon2.getLineOfSightVerticesIndices(polygon1.getVertex(L1), lineOfSight1);
         if (!success)
            return false;
         success = polygon2.getLineOfSightVerticesIndices(polygon1.getVertex(R1), lineOfSight2);
         if (!success)
            return false;

         if ((L2 == lineOfSight2[0]) && (R2 == lineOfSight1[1]))
         {
            done = true;
            break;
         }

         L2 = lineOfSight2[0];
         R2 = lineOfSight1[1];

         success = polygon1.getLineOfSightVerticesIndices(polygon2.getVertex(L2), lineOfSight1);
         if (!success)
            return false;
         success = polygon1.getLineOfSightVerticesIndices(polygon2.getVertex(R2), lineOfSight2);
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

      packConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);

      return true;
   }

   public static boolean findConnectingEdges(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, LineSegment2d connectingEdge1ToPack,
         LineSegment2d connectingEdge2Topack)
   {
      int[][] verticesIndices = new int[2][2];
      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, verticesIndices);
      if (!success)
         return false;

      packConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);
      return true;
   }

   private static void packConnectingEdges(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, LineSegment2d connectingEdge1ToPack,
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
         return ConvexPolygonTools.computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonP, polygonQ,
               intersectingPolygonToPack);
      }

      else if (polygonQ.hasExactlyTwoVertices())
      {
         return ConvexPolygonTools.computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonQ, polygonP,
               intersectingPolygonToPack);
      }

      if (polygonP.hasExactlyOneVertex() && polygonQ.hasAtLeastOneVertex())
      {
         return ConvexPolygonTools.computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonP, polygonQ, intersectingPolygonToPack);
      }

      else if (polygonQ.hasExactlyOneVertex())
      {
         return ConvexPolygonTools.computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonQ, polygonP, intersectingPolygonToPack);
      }

      // Find left most point on polygon1
      int currentPPolygonPointIndex = polygonP.getMinXIndex();
      int initialPolygonPIndex = polygonP.getMinXIndex();
      Point2d currentPolygonPPoint = polygonP.getVertex(currentPPolygonPointIndex);

      // Find left most point on polygon2
      int currentQPolygonPointIndex = polygonQ.getMinXIndex();
      int initialPolygonQIndex = polygonQ.getMinXIndex();
      Point2d currentPolygonQPoint = polygonQ.getVertex(currentQPolygonPointIndex);

      // At each of those two vertices, place a vertical line passing through it. Associate that line to the polygon to which the vertex belongs.
      Vector2d caliperForPolygonP = new Vector2d(0.0, 1.0);
      Vector2d caliperForPolygonQ = new Vector2d(0.0, 1.0);

      // determine which side polygon2's caliper line is on relative to polygon1's caliper line
      Point2d lineStart = new Point2d(currentPolygonPPoint);
      Point2d lineEnd = new Point2d(currentPolygonPPoint);
      lineEnd.add(caliperForPolygonP);
      boolean isOnLeft = GeometryTools.isPointOnLeftSideOfLine(currentPolygonQPoint, lineStart, lineEnd);
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
         Vector2d vectorToNextPointOnPolygonP = new Vector2d(polygonP.getNextVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.sub(polygonP.getVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.normalize();

         // +++JEP: Don't actually compute the angle! Just look at the dot products!
         //       double angleToNextPointOnPolygonP = caliperForPolygonP.angle(vectorToNextPointOnPolygonP); //Math.acos(vectorToNextPointOnPolygon1.getY());
         double dotProductToNextPointOnPolygonP = caliperForPolygonP.dot(vectorToNextPointOnPolygonP); // Math.acos(vectorToNextPointOnPolygon1.getY());

         // find angle from current to next point for polygon2
         Vector2d vectorToNextPointOnPolygonQ = new Vector2d(polygonQ.getNextVertex(currentQPolygonPointIndex));
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
         lineStart = new Point2d(currentPolygonPPoint);
         lineEnd = new Point2d(currentPolygonPPoint);
         lineEnd.add(caliperForPolygonP);
         isOnLeft = GeometryTools.isPointOnLeftSideOfLine(currentPolygonQPoint, lineStart, lineEnd);

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
         if (polygonP.isPointInside(polygonQ.getVertex(0)))
         {
            intersectingPolygonToPack.setAndUpdate(polygonQ);
         }

         if (polygonQ.isPointInside(polygonP.getVertex(0)))
         {
            intersectingPolygonToPack.setAndUpdate(polygonP);
         }
      }
      else
      {
         boolean success = ConvexPolygonTools.buildCommonPolygonFromBridges(bridgeIndicesP, bridgeIndicesQ, bridgeWasOnLeft, polygonP, polygonQ,
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
      Point2d[] intersection = polygonWithAtLeastTwoVertices.intersectionWith(polygonWithTwoVerticesAsLineSegment);

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

      Point2d linePStart = polygonP.getVertex(indexPStart);
      Point2d linePEnd = polygonP.getVertex(indexPEnd);

      Point2d lineQStart = polygonQ.getVertex(indexQStart);
      Point2d lineQEnd = polygonQ.getVertex(indexQEnd);

      int initialPPolygonStartIndex = indexPStart;
      int initialQPolygonStartIndex = indexQStart;

      boolean finished;
      do
      {
         finished = true;

         while (decrementP ^ GeometryTools.isPointOnLeftSideOfLine(lineQEnd, linePStart, linePEnd))
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

         while ((!decrementP) ^ GeometryTools.isPointOnLeftSideOfLine(linePEnd, lineQStart, lineQEnd))
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

         Point2d startP = polygonP.getVertex(startIndexP);
         Point2d endP = polygonP.getVertex(endIndexP);
         Point2d startQ = polygonQ.getVertex(startIndexQ);
         Point2d endQ = polygonQ.getVertex(endIndexQ);

         Point2d intersection = GeometryTools.getIntersectionBetweenTwoLines(startP, endP, startQ, endQ);

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



   public static ConvexPolygon2d shrinkInto(ConvexPolygon2d polygonP, Point2d referencePointInP, ConvexPolygon2d polygonQ)
   {
      if (polygonQ.hasAtLeastOneVertex() && !polygonQ.hasAtLeastThreeVertices())
      {
         return new ConvexPolygon2d(polygonQ);
      }

      ArrayList<Line2d> rays = new ArrayList<Line2d>();

      Point2d referencePointInPCopy = new Point2d(referencePointInP);

      int leftMostIndexOnPolygonQ = polygonQ.getMinXIndex();
      Point2d vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2d nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

      int leftMostIndexOnPolygonP = polygonP.getMinXIndex();
      Point2d vertexP = polygonP.getVertex(leftMostIndexOnPolygonP);
      int nextVertexPIndex = polygonP.getNextVertexIndex(leftMostIndexOnPolygonP);
      Point2d nextVertexP = polygonP.getVertex(nextVertexPIndex);

      forEachPolygonQ: for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         Vector2d edgeOnQ = new Vector2d(nextVertexQ.getX() - vertexQ.getX(), nextVertexQ.getY() - vertexQ.getY());

         int j = 0;
         while (j < polygonP.getNumberOfVertices())
         {
            Vector2d edgeOnP = new Vector2d(nextVertexP.getX() - vertexP.getX(), nextVertexP.getY() - vertexP.getY());
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
   public static Point2d[] intersection(LineSegment2d lineSegment, ConvexPolygon2d convexPolygon)
   {
      Point2d[] intersectingPoints = convexPolygon.intersectionWith(lineSegment);

      if (intersectingPoints == null)
         return null;

      Point2d[] ret = new Point2d[intersectingPoints.length];

      for (int i = 0; i < intersectingPoints.length; i++)
      {
         ret[i] = new Point2d(intersectingPoints[i]);
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

      combinedPolygonToPack.updateFramePoints();

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
      FramePoint2d[] intersectionPoints = polygonToCut.intersectionWith(cuttingLine);

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
            FramePoint2d vertex = polygonToCut.getFrameVertexUnsafe(index);
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
         polygonToCut.addVertices(intersectionPoints);
         polygonToCut.update();
         return numberOfVerticesRemoved;
      }
   }
}
