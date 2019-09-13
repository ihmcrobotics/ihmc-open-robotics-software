package us.ihmc.robotics.geometry;

import java.util.List;
import java.util.PriorityQueue;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ConvexPolygonTools
{
   private static final boolean DEBUG = false;

   /**
    * Assumes that the polygons are disjoint. Find the vertex indices corresponding to the end
    * points of the two connecting edges.
    *
    * @param polygon1 the first polygon
    * @param polygon2 the second polygon
    * @param verticesIndices in[2][2] contains the indexes of the connecting edges end points. The
    *           row index refers to which polygon the vertex index belongs to, whereas the column
    *           index refers to which connecting edge the vertex index belongs to. For example,
    *           vertexIndexes[0][1] is the index of the vertex of the first polygon, also end point
    *           the second connecting edge.
    * @return success (false = failed, true = succeeded)
    */
   public static boolean findConnectingEdgesVerticesIndexes(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, int[][] verticesIndices)
   {
      boolean success = false;

      if (polygon1.isEmpty() || polygon2.isEmpty())
         return false;

      if (polygon1.getNumberOfVertices() == 1 && polygon2.getNumberOfVertices() == 1)
      {
         verticesIndices[0][0] = 0;
         verticesIndices[0][1] = 0;
         verticesIndices[1][0] = 0;
         verticesIndices[1][1] = 0;
         return true;
      }

      if (polygon1.getNumberOfVertices() == 1)
      {
         verticesIndices[0][0] = 0;
         verticesIndices[0][1] = 0;
         int lineOfSightStartIndex = polygon2.lineOfSightStartIndex(polygon1.getVertex(0));
         int lineOfSightEndIndex = polygon2.lineOfSightEndIndex(polygon1.getVertex(0));
         success = lineOfSightStartIndex > -1 && lineOfSightEndIndex > -1;
         verticesIndices[1][0] = lineOfSightStartIndex;
         verticesIndices[1][1] = lineOfSightStartIndex;
         return success;
      }

      if (polygon2.getNumberOfVertices() == 1)
      {
         verticesIndices[1][0] = 0;
         verticesIndices[1][1] = 0;
         int lineOfSightStartIndex = polygon1.lineOfSightStartIndex(polygon2.getVertex(0));
         int lineOfSightEndIndex = polygon1.lineOfSightEndIndex(polygon2.getVertex(0));
         success = lineOfSightStartIndex > -1 && lineOfSightEndIndex > -1;
         verticesIndices[0][0] = lineOfSightStartIndex;
         verticesIndices[0][1] = lineOfSightStartIndex;
         return success;
      }

      // First pick a random vertex from polygon1
      Point2DReadOnly vertex = polygon1.getVertex(0);

      int lineOfSight1StartIndex, lineOfSight1EndIndex;
      int lineOfSight2StartIndex, lineOfSight2EndIndex;
      int L1, R1, L2, R2;

      // Then find its two line of sight points on polygon 2:
      if ((lineOfSight1StartIndex = polygon2.lineOfSightStartIndex(vertex)) == -1)
         return false;
      if ((lineOfSight1EndIndex = polygon2.lineOfSightEndIndex(vertex)) == -1)
         return false;

      L2 = lineOfSight1StartIndex;
      R2 = lineOfSight1EndIndex;

      // Then find the line of sight vertices on polygon 1:
      if ((lineOfSight1StartIndex = polygon1.lineOfSightStartIndex(polygon2.getVertex(R2))) == -1)
         return false;
      if ((lineOfSight1EndIndex = polygon1.lineOfSightEndIndex(polygon2.getVertex(R2))) == -1)
         return false;

      if ((lineOfSight2StartIndex = polygon1.lineOfSightStartIndex(polygon2.getVertex(L2))) == -1)
         return false;
      if ((lineOfSight2EndIndex = polygon1.lineOfSightEndIndex(polygon2.getVertex(L2))) == -1)
         return false;

      L1 = lineOfSight1StartIndex;
      R1 = lineOfSight2EndIndex;

      // Find the line of sight vertices back and forth between the two polygons until they are constant between two iterations.
      boolean done = false;
      while (!done)
      {
         if ((lineOfSight1EndIndex = polygon2.lineOfSightEndIndex(polygon1.getVertex(L1))) == -1)
            return false;

         if ((lineOfSight2StartIndex = polygon2.lineOfSightStartIndex(polygon1.getVertex(R1))) == -1)
            return false;

         if ((L2 == lineOfSight2StartIndex) && (R2 == lineOfSight1EndIndex))
         {
            done = true;
            break;
         }

         L2 = lineOfSight2StartIndex;
         R2 = lineOfSight1EndIndex;

         if ((lineOfSight1EndIndex = polygon1.lineOfSightEndIndex(polygon2.getVertex(L2))) == -1)
            return false;

         if ((lineOfSight2StartIndex = polygon1.lineOfSightStartIndex(polygon2.getVertex(R2))) == -1)
            return false;

         if ((L1 == lineOfSight2StartIndex) && (R1 == lineOfSight1EndIndex))
         {
            done = true;
            break;
         }

         L1 = lineOfSight2StartIndex;
         R1 = lineOfSight1EndIndex;
      }

      verticesIndices[0][0] = L1;
      verticesIndices[0][1] = R1;
      verticesIndices[1][0] = R2;
      verticesIndices[1][1] = L2;
      return true;
   }

   private final int[][] verticesIndices = new int[2][2];

   /**
    * Efficiently combines two Disjoint Polygons. Returns false if not disjoint.
    *
    * @param polygon1 ConvexPolygon2d
    * @param polygon2 ConvexPolygon2d
    * @param combinedPolygonToPack ConvexPolygon2d polygon in which we put the convex hull
    *           containing polygon1 and polygon2.
    * @param connectingEdge1ToPack LineSegment2d first connecting edge between polygon1 and
    *           polygon2.
    * @param connectingEdge2Topack LineSegment2d second connecting edge between polygon1 and
    *           polygon2.
    * @return true if succeeded, false if failed
    */
   public boolean combineDisjointPolygons(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, ConvexPolygon2DBasics combinedPolygonToPack,
                                          LineSegment2DBasics connectingEdge1ToPack, LineSegment2DBasics connectingEdge2Topack)
   {

      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, verticesIndices);
      if (!success)
         return false;

      combinedPolygonToPack.clear();
      polygon1.getVerticesInClockwiseOrder(verticesIndices[0][1], verticesIndices[0][0], combinedPolygonToPack);
      polygon2.getVerticesInClockwiseOrder(verticesIndices[1][0], verticesIndices[1][1], combinedPolygonToPack);
      combinedPolygonToPack.update();

      getConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);

      return true;
   }

   public boolean findConnectingEdges(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, LineSegment2DBasics connectingEdge1ToPack,
                                      LineSegment2DBasics connectingEdge2Topack)
   {
      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, verticesIndices);
      if (!success)
         return false;

      getConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2Topack, verticesIndices);
      return true;
   }

   protected static void getConnectingEdges(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, LineSegment2DBasics connectingEdge1ToPack,
                                            LineSegment2DBasics connectingEdge2ToPack, int[][] verticesIndices)
   {
      connectingEdge1ToPack.set(polygon1.getVertex(verticesIndices[0][0]), polygon2.getVertex(verticesIndices[1][0]));
      connectingEdge2ToPack.set(polygon2.getVertex(verticesIndices[1][1]), polygon1.getVertex(verticesIndices[0][1]));
   }

   private final Vector2D caliperForPolygonP = new Vector2D();
   private final Vector2D caliperForPolygonQ = new Vector2D();
   private final Point2D lineStart = new Point2D();
   private final Point2D lineEnd = new Point2D();
   private final TIntArrayList bridgeIndicesP = new TIntArrayList();
   private final TIntArrayList bridgeIndicesQ = new TIntArrayList();
   private final TIntArrayList bridgeWasOnLeft = new TIntArrayList();
   private final Vector2D vectorToNextPointOnPolygonP = new Vector2D();
   private final Vector2D vectorToNextPointOnPolygonQ = new Vector2D();

   /**
    * Computes the intersection of two convex polygons. For references see:
    * http://www.iro.umontreal.ca/~plante/compGeom/algorithm.html Returns null if the polygons do
    * not intersect Returns the inside polygon if the two polygons are inside one another.
    *
    * @param polygonP ConvexPolygon2d
    * @param polygonQ ConvexPolygon2d
    * @return ConvexPolygon2d Intersection of polygonP and polygonQ
    */
   public boolean computeIntersectionOfPolygons(ConvexPolygon2DReadOnly polygonP, ConvexPolygon2DReadOnly polygonQ,
                                                ConvexPolygon2DBasics intersectingPolygonToPack)
   {
      // return false if either polygon null
      if (polygonP == null || polygonP.isEmpty())
         return false;
      if (polygonQ == null || polygonQ.isEmpty())
         return false;

      if (polygonP.getNumberOfVertices() == 2 && polygonQ.getNumberOfVertices() >= 2)
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonP, polygonQ, intersectingPolygonToPack);
      }

      else if (polygonQ.getNumberOfVertices() == 2)
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(polygonQ, polygonP, intersectingPolygonToPack);
      }

      if (polygonP.getNumberOfVertices() == 1 && !polygonQ.isEmpty())
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonP, polygonQ, intersectingPolygonToPack);
      }

      else if (polygonQ.getNumberOfVertices() == 1)
      {
         return computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(polygonQ, polygonP, intersectingPolygonToPack);
      }

      // Find left most point on polygon1
      int currentPPolygonPointIndex = EuclidGeometryPolygonTools.findVertexIndex(polygonP, true, Bound.MIN, Bound.MIN);
      int initialPolygonPIndex = currentPPolygonPointIndex;
      Point2DReadOnly currentPolygonPPoint = polygonP.getVertex(currentPPolygonPointIndex);

      // Find left most point on polygon2
      int currentQPolygonPointIndex = EuclidGeometryPolygonTools.findVertexIndex(polygonQ, true, Bound.MIN, Bound.MIN);
      int initialPolygonQIndex = currentQPolygonPointIndex;
      Point2DReadOnly currentPolygonQPoint = polygonQ.getVertex(currentQPolygonPointIndex);

      // At each of those two vertices, place a vertical line passing through it. Associate that line to the polygon to which the vertex belongs.
      caliperForPolygonP.set(0.0, 1.0);
      caliperForPolygonQ.set(0.0, 1.0);

      // determine which side polygon2's caliper line is on relative to polygon1's caliper line
      lineStart.set(currentPolygonPPoint);
      lineEnd.set(currentPolygonPPoint);
      lineEnd.add(caliperForPolygonP);
      boolean isOnLeft = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(currentPolygonQPoint, lineStart, lineEnd);
      boolean wasOnLeft = isOnLeft;

      //    System.out.println("wasOnLeft = " + wasOnLeft);

      boolean gotAroundPOnce = false;
      boolean gotAroundQOnce = false;
      boolean DONE = false;

      int bridgeCount = 0;
      bridgeIndicesP.reset();
      bridgeIndicesQ.reset();
      bridgeWasOnLeft.reset();

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
         vectorToNextPointOnPolygonP.set(polygonP.getNextVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.sub(polygonP.getVertex(currentPPolygonPointIndex));
         vectorToNextPointOnPolygonP.normalize();

         // +++JEP: Don't actually compute the angle! Just look at the dot products!
         //       double angleToNextPointOnPolygonP = caliperForPolygonP.angle(vectorToNextPointOnPolygonP); //Math.acos(vectorToNextPointOnPolygon1.getY());
         double dotProductToNextPointOnPolygonP = caliperForPolygonP.dot(vectorToNextPointOnPolygonP); // Math.acos(vectorToNextPointOnPolygon1.getY());

         // find angle from current to next point for polygon2
         vectorToNextPointOnPolygonQ.set(polygonQ.getNextVertex(currentQPolygonPointIndex));
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
         lineStart.set(currentPolygonPPoint);
         lineEnd.set(currentPolygonPPoint);
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
               bridgeWasOnLeft.add(wasOnLeft ? 1 : 0);

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
            intersectingPolygonToPack.set(polygonQ);
         }

         if (polygonQ.isPointInside(polygonP.getVertex(0)))
         {
            intersectingPolygonToPack.set(polygonP);
         }
      }
      else
      {
         boolean success = buildCommonPolygonFromBridges(bridgeIndicesP, bridgeIndicesQ, bridgeWasOnLeft, polygonP, polygonQ, intersectingPolygonToPack);
         if (!success)
         {
            intersectingPolygonToPack.clearAndUpdate();
            return false;
         }
      }

      // Merge the inner chains to determine intersection
      return true;
   }

   private static boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(ConvexPolygon2DReadOnly polygonWithExactlyOneVertex,
                                                                                       ConvexPolygon2DReadOnly otherPolygon,
                                                                                       ConvexPolygon2DBasics intersectingPolygon)
   {
      if (otherPolygon.pointIsOnPerimeter(polygonWithExactlyOneVertex.getVertex(0)))
      {
         intersectingPolygon.set(polygonWithExactlyOneVertex);
         return false;
      }
      else
      {
         intersectingPolygon.clearAndUpdate();
         return false;
      }
   }

   private final LineSegment2D polygonWithTwoVerticesAsLineSegment = new LineSegment2D();
   private final Point2D intersection1 = new Point2D();
   private final Point2D intersection2 = new Point2D();

   private boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(ConvexPolygon2DReadOnly polygonWithExactlyTwoVertices,
                                                                                                                  ConvexPolygon2DReadOnly polygonWithAtLeastTwoVertices,
                                                                                                                  ConvexPolygon2DBasics intersectingPolygon)
   {
      polygonWithTwoVerticesAsLineSegment.set(polygonWithExactlyTwoVertices.getVertex(0), polygonWithExactlyTwoVertices.getVertex(1));
      int intersections = polygonWithAtLeastTwoVertices.intersectionWith(polygonWithTwoVerticesAsLineSegment, intersection1, intersection2);

      if (intersections == 0)
      {
         intersectingPolygon.clearAndUpdate();
         return false;
      }
      else if (intersections == 1)
      {
         intersectingPolygon.clear();
         intersectingPolygon.addVertex(intersection1);
         intersectingPolygon.update();
         return true;
      }
      else
      {
         intersectingPolygon.clear();
         intersectingPolygon.addVertex(intersection1);
         intersectingPolygon.addVertex(intersection2);
         intersectingPolygon.update();
         return true;
      }
   }

   private static boolean findCrossingIndices(boolean decrementP, int bridgeIndexForPolygonP, int bridgeIndexForPolygonQ, ConvexPolygon2DReadOnly polygonP,
                                              ConvexPolygon2DReadOnly polygonQ, TIntArrayList indicesToPack)
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
               return false; // No intersection. Prevent infinite loop.
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
               return false; // No intersection. Prevent infinite loop.
         }

      }
      while (!finished);

      indicesToPack.reset();
      indicesToPack.add(indexPStart);
      indicesToPack.add(indexPEnd);
      indicesToPack.add(indexQStart);
      indicesToPack.add(indexQEnd);
      return true;
   }

   private final Point2D intersection = new Point2D();

   protected boolean constructPolygonForIntersection(List<TIntArrayList> crossingIndices, ConvexPolygon2DReadOnly polygonP, ConvexPolygon2DReadOnly polygonQ,
                                                     ConvexPolygon2DBasics intersectingPolygonToPack)
   {
      int startIndexP1 = crossingIndices.get(0).get(0);
      int endIndexP1 = crossingIndices.get(0).get(1);

      int startIndexQ1 = crossingIndices.get(0).get(2);
      int endIndexQ1 = crossingIndices.get(0).get(3);

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
      for (int i = 0; i < crossingIndices.size(); i++)
      {
         int startIndexP = crossingIndices.get(i).get(0);
         int endIndexP = crossingIndices.get(i).get(1);

         int startIndexQ = crossingIndices.get(i).get(2);
         int endIndexQ = crossingIndices.get(i).get(3);

         Point2DReadOnly startP = polygonP.getVertex(startIndexP);
         Point2DReadOnly endP = polygonP.getVertex(endIndexP);
         Point2DReadOnly startQ = polygonQ.getVertex(startIndexQ);
         Point2DReadOnly endQ = polygonQ.getVertex(endIndexQ);

         boolean success = EuclidCoreMissingTools.intersectionBetweenTwoLine2Ds(startP, endP, startQ, endQ, intersection);
         if (!success)
         {
            if (DEBUG)
            {
               PrintTools.error("intersection is null in constructPolygonForIntersection!. startP = " + startP + ", endP = " + endP + ", startQ = " + startQ
                     + ", endQ = " + endQ);
               PrintTools.error("startIndexP = " + startIndexP + ", endIndexP = " + endIndexP);
               PrintTools.error("Returning null polygon");
            }

            intersectingPolygonToPack.clear();
            intersectingPolygonToPack.update();
            return false;
         }

         intersectingPolygonToPack.addVertex(intersection);

         if (incrementPNext)
         {
            int indexP = crossingIndices.get(i).get(1); // endIndexP;
            int indexPNext = crossingIndices.get((i + 1) % crossingIndices.size()).get(0);

            //          System.out.println("indexP = " + indexP + ", indexPNext = " + indexPNext);

            while (indexP != indexPNext)
            {
               intersectingPolygonToPack.addVertex(polygonP.getVertex(indexP));
               indexP = polygonP.getNextVertexIndex(indexP);
            }
         }
         else
         {
            int indexQ = crossingIndices.get(i).get(3); // endIndexQ;
            int indexQNext = crossingIndices.get((i + 1) % crossingIndices.size()).get(2);

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

   private final RecyclingArrayList<TIntArrayList> crossingIndices = new RecyclingArrayList<>(TIntArrayList.class);

   private boolean buildCommonPolygonFromBridges(TIntArrayList bridgeIndicesP, TIntArrayList bridgeIndicesQ, TIntArrayList bridgeWasOnLeft,
                                                 ConvexPolygon2DReadOnly polygonP, ConvexPolygon2DReadOnly polygonQ,
                                                 ConvexPolygon2DBasics intersectingPolygonToPack)
   {
      crossingIndices.clear();
      for (int i = 0; i < bridgeIndicesP.size(); i++)
      {
         // find intersection for bridge
         int bridgeIndexForPolygonP = bridgeIndicesP.get(i);
         int bridgeIndexForPolygonQ = bridgeIndicesQ.get(i);

         // for each bridge, compute the intersection points
         TIntArrayList indices = crossingIndices.add();
         boolean success = findCrossingIndices(bridgeWasOnLeft.get(i) == 1, bridgeIndexForPolygonP, bridgeIndexForPolygonQ, polygonP, polygonQ, indices);

         if (!success)
         {
            intersectingPolygonToPack.clearAndUpdate();
            return false; // No intersection.
         }
      }

      PriorityQueue p;

      boolean success = constructPolygonForIntersection(crossingIndices, polygonP, polygonQ, intersectingPolygonToPack);
      return success;
   }

   private final Point2D referencePointInPCopy = new Point2D();
   private final RecyclingArrayList<Line2D> rays = new RecyclingArrayList<>(Line2D.class);
   private final Vector2D edgeOnQ = new Vector2D();
   private final Vector2D edgeOnP = new Vector2D();
   private final ConvexPolygonConstructorFromInteriorOfRays convexPolygonConstructorFromInteriorOfRays = new ConvexPolygonConstructorFromInteriorOfRays();

   public boolean shrinkInto(ConvexPolygon2DReadOnly polygonP, Point2DReadOnly referencePointInP, ConvexPolygon2DReadOnly polygonQ,
                             ConvexPolygon2D polygonToPack)
   {
      if (!polygonQ.isEmpty() && polygonQ.getNumberOfVertices() < 3)
      {
         polygonToPack.set(polygonQ);
         return true;
      }

      rays.clear();
      referencePointInPCopy.set(referencePointInP);

      int leftMostIndexOnPolygonQ = EuclidGeometryPolygonTools.findVertexIndex(polygonQ, true, Bound.MIN, Bound.MIN);
      Point2DReadOnly vertexQ = polygonQ.getVertex(leftMostIndexOnPolygonQ);
      int nextVertexQIndex = polygonQ.getNextVertexIndex(leftMostIndexOnPolygonQ);
      Point2DReadOnly nextVertexQ = polygonQ.getVertex(nextVertexQIndex);

      int leftMostIndexOnPolygonP = EuclidGeometryPolygonTools.findVertexIndex(polygonP, true, Bound.MIN, Bound.MIN);
      Point2DReadOnly vertexP = polygonP.getVertex(leftMostIndexOnPolygonP);
      int nextVertexPIndex = polygonP.getNextVertexIndex(leftMostIndexOnPolygonP);
      Point2DReadOnly nextVertexP = polygonP.getVertex(nextVertexPIndex);

      forEachPolygonQ: for (int i = 0; i < polygonQ.getNumberOfVertices(); i++)
      {
         edgeOnQ.set(nextVertexQ.getX() - vertexQ.getX(), nextVertexQ.getY() - vertexQ.getY());

         int j = 0;
         while (j < polygonP.getNumberOfVertices())
         {
            edgeOnP.set(nextVertexP.getX() - vertexP.getX(), nextVertexP.getY() - vertexP.getY());
            double crossProduct = edgeOnQ.getX() * edgeOnP.getY() - edgeOnP.getX() * edgeOnQ.getY();
            if (crossProduct <= 0.0)
            {
               referencePointInPCopy.setX(referencePointInP.getX() + vertexQ.getX() - vertexP.getX());
               referencePointInPCopy.setY(referencePointInP.getY() + vertexQ.getY() - vertexP.getY());
               rays.add().set(referencePointInPCopy, edgeOnQ);

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

      return convexPolygonConstructorFromInteriorOfRays.constructFromInteriorOfRays(rays, polygonToPack);
   }

   public boolean combineDisjointPolygons(FrameConvexPolygon2DReadOnly polygon1, FrameConvexPolygon2DReadOnly polygon2,
                                          FrameConvexPolygon2DBasics combinedPolygonToPack, FrameLineSegment2DBasics connectingEdge1ToPack,
                                          FrameLineSegment2DBasics connectingEdge2ToPack)
   {
      combinedPolygonToPack.clear(polygon1.getReferenceFrame());
      combinedPolygonToPack.checkReferenceFrameMatch(connectingEdge1ToPack);
      combinedPolygonToPack.checkReferenceFrameMatch(connectingEdge2ToPack);

      boolean success = combineDisjointPolygons((ConvexPolygon2DReadOnly) polygon1, (ConvexPolygon2DReadOnly) polygon2,
                                                (ConvexPolygon2DBasics) combinedPolygonToPack, (LineSegment2DBasics) connectingEdge1ToPack,
                                                (LineSegment2DBasics) connectingEdge2ToPack);

      if (!success)
         return false;

      //      combinedPolygonToPack.updateFramePoints();
      combinedPolygonToPack.update();

      return true;
   }

   public static ConvexPolygonCropResult cutPolygonToLeftOfLine(ConvexPolygon2DReadOnly polygonToCrop,
                                                                Line2DReadOnly cuttingLine,
                                                                ConvexPolygon2DBasics croppedPolygonToPack)
   {
      return cutPolygonToLeftOfLine(polygonToCrop, cuttingLine, croppedPolygonToPack, new Point2D(), new Point2D());
   }

   public static ConvexPolygonCropResult cutPolygonToLeftOfLine(ConvexPolygon2DReadOnly polygonToCrop,
                                                                Line2DReadOnly cuttingLine,
                                                                ConvexPolygon2DBasics croppedPolygonToPack,
                                                                Point2DBasics firstIntersectionToPack,
                                                                Point2DBasics secondIntersectionToPack)
   {
      if (polygonToCrop.isEmpty())
      {
         croppedPolygonToPack.clearAndUpdate();
         return ConvexPolygonCropResult.REMOVE_ALL;
      }

      Vector2D upDirection = new Vector2D(cuttingLine.getDirection());
      upDirection.normalize();
      upDirection.set(-upDirection.getY(), upDirection.getX());

      int intersectionCount = EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(cuttingLine.getPoint(),
                                                                                                     cuttingLine.getDirection(),
                                                                                                     polygonToCrop.getVertexBufferView(),
                                                                                                     polygonToCrop.getNumberOfVertices(),
                                                                                                     polygonToCrop.isClockwiseOrdered(),
                                                                                                     firstIntersectionToPack,
                                                                                                     secondIntersectionToPack);
      LogTools.trace("Intersection count: {}", intersectionCount);
      boolean vertex0IsAbove = EuclidGeometryTools.isPoint2DInFrontOfRay2D(polygonToCrop.getVertex(0), cuttingLine.getPoint(), upDirection);
      if (intersectionCount == 0)
      {
         if (vertex0IsAbove)
         {
            croppedPolygonToPack.set(polygonToCrop);
            return ConvexPolygonCropResult.KEEP_ALL;
         }
         else
         {
            croppedPolygonToPack.clearAndUpdate();
            return ConvexPolygonCropResult.REMOVE_ALL;
         }
      }
      else if (intersectionCount == 1)
      {
         // firstIntersectionToPack is packed with only intersection
         if (polygonToCrop.getNumberOfVertices() > 1)
         {
            // isPoint2DInFrontOfRay2D returns true for on as well. Check any two vertices. One is on the line.
            boolean isOnOrAboveTwo = EuclidGeometryTools.isPoint2DInFrontOfRay2D(polygonToCrop.getVertex(1), cuttingLine.getPoint(), upDirection);

            if (vertex0IsAbove && isOnOrAboveTwo)
            {
               croppedPolygonToPack.set(polygonToCrop);
               return ConvexPolygonCropResult.KEEP_ALL;
            }
            else
            {
               croppedPolygonToPack.clearAndUpdate();
               return ConvexPolygonCropResult.REMOVE_ALL;
            }
         }
         else
         {
            croppedPolygonToPack.clearAndUpdate();
            return ConvexPolygonCropResult.REMOVE_ALL;
         }
      }
      else
      {
         if (intersectionCount != 2)
            throw new RuntimeException("Should only be possible for 2 intersections with a convex polygon.");

         if (polygonToCrop.getNumberOfVertices() < 3)
         {
            croppedPolygonToPack.set(polygonToCrop);
            return ConvexPolygonCropResult.KEEP_ALL;
         }

         if (polygonToCrop.getNumberOfVertices() < 3)
            throw new RuntimeException("Two intersections only possible at this point only for convex polygon of size 3 or greater.");

         // find vertex after intersections
         int vertexAfterIntersectionOne = -1;
         int vertexAfterIntersectionTwo = -1;
         for (int i = 0; i < polygonToCrop.getNumberOfVertices(); i++) // loop over vertices
         {
            // is first, second intersection point on segment (colinear) with v(i)(i+1)
            int nextIndex = EuclidGeometryPolygonTools.next(i, polygonToCrop.getNumberOfVertices());

            Point2DReadOnly intersectionPointToCheck = vertexAfterIntersectionOne >= 0 ? secondIntersectionToPack : firstIntersectionToPack;
            boolean metIntersection = EuclidGeometryTools.isPoint2DOnLineSegment2D(intersectionPointToCheck,
                                                                                   polygonToCrop.getVertex(i),
                                                                                   polygonToCrop.getVertex(nextIndex));
            if (metIntersection)
            {
               if (vertexAfterIntersectionOne == -1)
               {
                  vertexAfterIntersectionOne = nextIndex;
               }
               else // at second intersection
               {
                  if (i == vertexAfterIntersectionOne) // this is a line intersection colinear with this edge
                  {
                     // if any below, keep intersection segment
                     for (int j = 0; j < polygonToCrop.getNumberOfVertices(); j++)
                     {
                        if (!EuclidGeometryTools.isPoint2DInFrontOfRay2D(polygonToCrop.getVertex(j), cuttingLine.getPoint(), upDirection))
                        {
                           croppedPolygonToPack.clear();
                           croppedPolygonToPack.addVertex(firstIntersectionToPack);
                           croppedPolygonToPack.addVertex(secondIntersectionToPack);
                           croppedPolygonToPack.update();
                           return ConvexPolygonCropResult.CUT;
                        }
                     }
                     // else keep all
                     croppedPolygonToPack.set(polygonToCrop);
                     return ConvexPolygonCropResult.KEEP_ALL;
                  }
                  else
                  {
                     vertexAfterIntersectionTwo = nextIndex;
                     break; // optimization
                  }
               }
            }
         }

         croppedPolygonToPack.clear();
         for (int i = 0; i < polygonToCrop.getNumberOfVertices(); i++) // loop over vertices
         {
            int nextIndex = EuclidGeometryPolygonTools.next(i, polygonToCrop.getNumberOfVertices());

            if (vertex0IsAbove || i > 0)
            {
               LogTools.debug("Adding v({})", i);
               croppedPolygonToPack.addVertex(polygonToCrop.getVertex(i));
            }

            if (vertexAfterIntersectionOne == nextIndex) // encounter 1st intersection, decisions to be made
            {
               LogTools.debug("Adding i(0)");
               croppedPolygonToPack.addVertex(firstIntersectionToPack);
               if (vertex0IsAbove) // traverse from i(0) to i(1)
               {
                  LogTools.debug("Adding i(1)");
                  croppedPolygonToPack.addVertex(secondIntersectionToPack);
                  i = vertexAfterIntersectionTwo - 1; // cut across polygon; the for loop i++ will immediately add 1 more
               }
            }
            else if (vertexAfterIntersectionTwo == nextIndex) // here, this is always the last to add
            {
               LogTools.debug("Adding i(1)");
               croppedPolygonToPack.addVertex(secondIntersectionToPack);
               break;
            }
         }
      }

      return ConvexPolygonCropResult.CUT;
   }

   public static int cutPolygonWithLine(FrameLine2DReadOnly cuttingLine, FixedFrameConvexPolygon2DBasics polygonToCut,
                                        FrameConvexPolygonWithLineIntersector2d lineIntersector2d, RobotSide sideOfLineToCut)
   {
      lineIntersector2d.intersectWithLine(polygonToCut, cuttingLine);

      if (lineIntersector2d.getIntersectionResult() == FrameConvexPolygonWithLineIntersector2d.IntersectionResult.NO_INTERSECTION
            || lineIntersector2d.getIntersectionResult() == FrameConvexPolygonWithLineIntersector2d.IntersectionResult.POINT_INTERSECTION)
      {
         return -1;
      }
      else
      {
         int numberOfVerticesRemoved = 0;
         int index = 0;
         while (index < polygonToCut.getNumberOfVertices())
         {
            Point2DReadOnly vertex = polygonToCut.getVertex(index);
            if (cuttingLine.isPointOnSideOfLine(vertex, sideOfLineToCut == RobotSide.LEFT))
            {
               polygonToCut.removeVertex(index);
               polygonToCut.update();
               numberOfVerticesRemoved++;
            }
            else
            {
               index++;
            }
         }
         polygonToCut.addVertex(lineIntersector2d.getIntersectionPointOne());
         polygonToCut.addVertex(lineIntersector2d.getIntersectionPointTwo());
         polygonToCut.update();
         return numberOfVerticesRemoved;
      }
   }

   public int cutPolygonWithLine(FrameLine2DReadOnly cuttingLine, FixedFrameConvexPolygon2DBasics polygonToCut, RobotSide sideOfLineToCut)
   {
      cuttingLine.checkReferenceFrameMatch(polygonToCut);
      return cutPolygonWithLine(cuttingLine, (ConvexPolygon2DBasics) polygonToCut, sideOfLineToCut);
   }

   // TODO Needs to be extracted to Euclid.
   public static int cutPolygonWithLine(Line2DReadOnly cuttingLine,
                                        ConvexPolygon2DBasics polygonToCut,
                                        RobotSide sideOfLineToCut,
                                        Point2D intersectionPoint1,
                                        Point2D intersectionPoint2)
   {
      int intersectionPoints = polygonToCut.intersectionWith(cuttingLine, intersectionPoint1, intersectionPoint2);

      if (intersectionPoints < 2)
      {
         return -1;
      }
      else
      {
         int numberOfVerticesRemoved = 0;
         int index = 0;
         while (index < polygonToCut.getNumberOfVertices())
         {
            Point2DReadOnly vertex = polygonToCut.getVertex(index);
            if (cuttingLine.isPointOnSideOfLine(vertex, sideOfLineToCut == RobotSide.LEFT))
            {
               polygonToCut.removeVertex(index);
               polygonToCut.update();
               numberOfVerticesRemoved++;
            }
            else
            {
               index++;
            }
         }
         polygonToCut.addVertex(intersectionPoint1);
         polygonToCut.addVertex(intersectionPoint2);
         polygonToCut.update();
         return numberOfVerticesRemoved;
      }
   }

   private final Point2D intersectionPoint1 = new Point2D();
   private final Point2D intersectionPoint2 = new Point2D();

   public int cutPolygonWithLine(Line2DReadOnly cuttingLine, ConvexPolygon2DBasics polygonToCut, RobotSide sideOfLineToCut)
   {
      return cutPolygonWithLine(cuttingLine, polygonToCut, sideOfLineToCut, intersectionPoint1, intersectionPoint2);
   }

   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It
    * is conservative in the sense, that the modified polygon will be contained in the original
    * polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(ConvexPolygon2DBasics polygon, int desiredVertices)
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
         for (int i = 1; i < vertices + 1; i++)
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
            idx1 = vertices - 1;
            idx2 = 0;
         }
         else
         {
            idx1 = removeVertex;
            idx2 = removeVertex - 1;
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

      int numberOfFailedIterations = 0;

      while (vertices < desiredVertices)
      {
         int index = -1;
         double longestEdgeLength = Double.NEGATIVE_INFINITY;
         Point2DReadOnly lastVertex = polygon.getVertex(0);
         for (int i = 1; i < vertices + 1; i++)
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
            idx1 = vertices - 1;
            idx2 = 0;
         }
         else
         {
            idx1 = index - 1;
            idx2 = index;
         }

         Point2DReadOnly vertexA = polygon.getVertex(idx1);
         Point2DReadOnly vertexB = polygon.getVertex(idx2);
         double xNew = (vertexA.getX() + vertexB.getX()) / 2.0;
         double yNew = (vertexA.getY() + vertexB.getY()) / 2.0;
         double dx = -(vertexB.getY() - vertexA.getY());
         double dy = (vertexB.getX() - vertexA.getX());
         double length = Math.sqrt(EuclidCoreTools.normSquared(dx, dy));
         double scale = 1.0e-7 / length;
         xNew += dx * scale;
         yNew += dy * scale;
         
         polygon.addVertex(xNew, yNew);
         polygon.update();

         int verticesPreviousValue = vertices;
         vertices = polygon.getNumberOfVertices();

         if (vertices == verticesPreviousValue)
            numberOfFailedIterations++;
         else
            numberOfFailedIterations = 0;

         if (numberOfFailedIterations >= 5)
         {
            LogTools.warn("Not able to increase number of polygon vertices.");
            break;
         }
      }
   }

   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It
    * is conservative in the sense, that the modified polygon will be contained in the original
    * polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(FixedFrameConvexPolygon2DBasics polygon, int desiredVertices)
   {
      limitVerticesConservative((ConvexPolygon2DBasics) polygon, desiredVertices);
   }

   private final ConvexPolygon2D intersectingPolygonToPack = new ConvexPolygon2D();
   private final int[] v1Tangents = new int[2];
   private final int[] v2Tangents = new int[2];
   private final int[] updatedIndices = new int[4];

   /**
    * Finds the minimum distance between two convex polygons Taken from
    * http://cygnus-x1.cs.duke.edu/~edels/Papers/1985-J-02-ComputingExtremeDistances.pdf
    *
    * @return Two points, one from each polygon, between which is the minimum distance between the
    *         two polygons
    */
   public void computeMinimumDistancePoints(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, double epsilon, Point2DBasics point1ToPack,
                                            Point2DBasics point2ToPack)
   {
      // TODO Do something more clever than actually computing the intersection there!
      if (computeIntersectionOfPolygons(polygon1, polygon2, intersectingPolygonToPack))
      {
         throw new RuntimeException("Cannot compute minimum distance between intersecting polygons.");
      }

      if ((polygon1.getNumberOfVertices() < 3) || (polygon2.getNumberOfVertices() < 3))
      {
         throw new RuntimeException("Polygon inputs are degenerate.");
      }

      findStartAndEndTangents(polygon2.getVertex(0), polygon1, epsilon, v1Tangents);
      findStartAndEndTangents(polygon1.getVertex(0), polygon2, epsilon, v2Tangents);

      int v1Start = v1Tangents[0];
      int v1End = v1Tangents[1];
      int v2Start = v2Tangents[0];
      int v2End = v2Tangents[1];

      binaryElimination(polygon1, polygon2, v1Start, v1End, v2Start, v2End, epsilon, updatedIndices);
      v1Start = updatedIndices[0];
      v1End = updatedIndices[1];
      v2Start = updatedIndices[2];
      v2End = updatedIndices[3];

      getClosestPointsFromRemainingEdgesAndVertices(polygon1, polygon2, v1Start, v1End, v2Start, v2End, point1ToPack, point2ToPack);
   }

   public void computeMinimumDistancePoints(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, Point2DBasics point1ToPack,
                                            Point2DBasics point2ToPack)
   {
      computeMinimumDistancePoints(polygon1, polygon2, .01, point1ToPack, point2ToPack);
   }

   // TODO potentially implement [Chazelle and Dobkin] to get logarithmic running time for computeMinimumDistancePoints (though it would actually be log^2 in current
   // implementation, since binaryElimination, which has is O(log(n)) uses this method in each loop)

   private final Vector2D tangent1 = new Vector2D();
   private final Vector2D tangent2 = new Vector2D();

   /**
    * Finds the indices of the vertices of the polygon that form tangent lines to the polygon with
    * the parameter point
    *
    * @return The desired indices, ordered such that they form a range that includes all vertices
    *         visible from the parameter point; if there are more than two only returns the two
    *         necessary to specify this range
    */
   private void findStartAndEndTangents(Point2DReadOnly point, ConvexPolygon2DReadOnly polygon, double epsilon, int[] tangetsToPack)
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
      tangent1.set(polygon.getVertex(tangentIndex1).getX() - point.getX(), polygon.getVertex(tangentIndex1).getY() - point.getY());

      vIndex++;
      vIndex %= polygon.getNumberOfVertices();

      while (!pointMakesTangentToPolygon(polygon, point, vIndex, epsilon))
      {
         vIndex++;
         vIndex %= polygon.getNumberOfVertices();
      }

      tangentIndex2 = vIndex;
      tangent2.set(polygon.getVertex(tangentIndex2).getX() - point.getX(), polygon.getVertex(tangentIndex2).getY() - point.getY());

      if (tangent1.angle(tangent2) >= 0)
      {
         tangetsToPack[0] = tangentIndex1;
         tangetsToPack[1] = tangentIndex2;
      }
      else
      {
         tangetsToPack[0] = tangentIndex2;
         tangetsToPack[1] = tangentIndex1;
      }

   }

   private final Vector2D base = new Vector2D();
   private final Vector2D first = new Vector2D();
   private final Vector2D second = new Vector2D();

   /**
    * Uses the fact that if a line passes through a vertex of a convex polygon, the angles to the
    * adjacent edges are going to be in opposite directions
    *
    * @return Whether or not the line including the point and vertex is tangent to the polygon
    */
   private boolean pointMakesTangentToPolygon(ConvexPolygon2DReadOnly polygon, Point2DReadOnly point, int vertexIndex, double epsilon)
   {
      Point2DReadOnly vertex = polygon.getVertex(vertexIndex);
      Point2DReadOnly previous = polygon.getPreviousVertex(vertexIndex);
      Point2DReadOnly next = polygon.getNextVertex(vertexIndex);

      base.set(point.getX() - vertex.getX(), point.getY() - vertex.getY());
      first.set(previous.getX() - vertex.getX(), previous.getY() - vertex.getY());
      second.set(next.getX() - vertex.getX(), next.getY() - vertex.getY());
      double firstAngle = base.angle(first);
      double secondAngle = base.angle(second);

      if (firstAngle * secondAngle >= 0)
      { // if both angles have the same sign, the line does not pass through the polygon
         return true;
      }

      if (MathTools.epsilonEquals(firstAngle, 0, epsilon) || MathTools.epsilonEquals(secondAngle, 0, epsilon))
      { // if either angle is close to 0, assume floating point arithmetic error
         return true;
      }

      return false;
   }

   /**
    * Checks if index is within range; if low is greater than high, this implies a modularly
    * cyclical range
    *
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

   private final Vector2D m = new Vector2D();
   private final Vector2D mReversed = new Vector2D();
   private final Vector2D edge1A = new Vector2D();
   private final Vector2D edge1B = new Vector2D();
   private final Vector2D edge2A = new Vector2D();
   private final Vector2D edge2B = new Vector2D();
   private final int[] range1 = new int[2];
   private final int[] range2 = new int[2];
   private final int[] updatedIndicesInBinaryElimination = new int[4];

   /**
    * Eliminates vertices and return a range for each polygon, each of which comprises of at most
    * two vertices
    */
   private void binaryElimination(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, int v1Start, int v1End, int v2Start, int v2End,
                                  double epsilon, int[] indicesToPack)
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

         m.set(v2Median.getX() - v1Median.getX(), v2Median.getY() - v1Median.getY());
         mReversed.set(v1Median.getX() - v2Median.getX(), v1Median.getY() - v2Median.getY());

         int edge1AStart = ((v1MedianIndex + numberOfVertices1 - 1) % numberOfVertices1);
         int edge1BEnd = (v1MedianIndex + 1) % numberOfVertices1;
         int edge2BStart = ((v2MedianIndex + numberOfVertices2 - 1) % numberOfVertices2);
         int edge2AEnd = (v2MedianIndex + 1) % numberOfVertices2;
         edge1A.set(polygon1.getVertex(edge1AStart).getX() - v1Median.getX(), polygon1.getVertex(edge1AStart).getY() - v1Median.getY());
         edge1B.set(polygon1.getVertex(edge1BEnd).getX() - v1Median.getX(), polygon1.getVertex(edge1BEnd).getY() - v1Median.getY());
         edge2A.set(polygon2.getVertex(edge2AEnd).getX() - v2Median.getX(), polygon2.getVertex(edge2AEnd).getY() - v2Median.getY());
         edge2B.set(polygon2.getVertex(edge2BStart).getX() - v2Median.getX(), polygon2.getVertex(edge2BStart).getY() - v2Median.getY());

         // see diagram 3.2 in [Edelsbrunner]
         double angle1A = m.angle(edge1A); // A' in diagram
         double angle1B = edge1B.angle(m); // A'' in diagram
         double angle2A = edge2A.angle(mReversed); // B' in diagram
         double angle2B = mReversed.angle(edge2B); // B'' in diagram

         findStartAndEndTangents(v2Median, polygon1, epsilon, range1);
         findStartAndEndTangents(v1Median, polygon2, epsilon, range2);

         angle1A = MathTools.epsilonEquals(angle1A, 0, 0.01) ? 0 : angle1A;
         angle1B = MathTools.epsilonEquals(angle1B, 0, 0.01) ? 0 : angle1B;
         angle2A = MathTools.epsilonEquals(angle2A, 0, 0.01) ? 0 : angle2A;
         angle2B = MathTools.epsilonEquals(angle2B, 0, 0.01) ? 0 : angle2B;

         angle1A += ((angle1A <= 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle1B += ((angle1B <= 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle2A += ((angle2A <= 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;
         angle2B += ((angle2B <= 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;

         angle1A += ((angle1A <= 0) && (angle1B <= 0) && (angle1A < angle1B)) ? 2 * Math.PI : 0;
         angle1B += ((angle1A <= 0) && (angle1B <= 0) && (angle1B < angle1A)) ? 2 * Math.PI : 0;
         angle2A += ((angle2A <= 0) && (angle2B <= 0) && (angle2A < angle2B)) ? 2 * Math.PI : 0;
         angle2B += ((angle2A <= 0) && (angle2B <= 0) && (angle2B < angle2A)) ? 2 * Math.PI : 0;

         if ((v1Start == v1End) || (v2Start == v2End))
         {
            binaryEliminationCase1(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2,
                                   updatedIndicesInBinaryElimination);
            v1Start = updatedIndicesInBinaryElimination[0];
            v1End = updatedIndicesInBinaryElimination[1];
            v2Start = updatedIndicesInBinaryElimination[2];
            v2End = updatedIndicesInBinaryElimination[3];
         }
         else if ((v1End - v1Start + numberOfVertices1) % numberOfVertices1 == 1)
         {
            binaryEliminationCase2(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2,
                                   updatedIndicesInBinaryElimination);
            v1Start = updatedIndicesInBinaryElimination[0];
            v1End = updatedIndicesInBinaryElimination[1];
            v2Start = updatedIndicesInBinaryElimination[2];
            v2End = updatedIndicesInBinaryElimination[3];
         }
         else if ((v2End - v2Start + numberOfVertices2) % numberOfVertices2 == 1)
         {
            binaryEliminationCase2(angle2A, angle2B, angle1A, angle1B, v2End, v2MedianIndex, v2Start, v1End, v1MedianIndex, v1Start, polygon1, polygon2,
                                   updatedIndicesInBinaryElimination);
            v2End = updatedIndicesInBinaryElimination[0];
            v2Start = updatedIndicesInBinaryElimination[1];
            v1End = updatedIndicesInBinaryElimination[2];
            v1Start = updatedIndicesInBinaryElimination[3];
         }
         else
         {
            binaryEliminationCase3(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End,
                                   updatedIndicesInBinaryElimination);
            v1Start = updatedIndicesInBinaryElimination[0];
            v1End = updatedIndicesInBinaryElimination[1];
            v2Start = updatedIndicesInBinaryElimination[2];
            v2End = updatedIndicesInBinaryElimination[3];
         }
      }

      indicesToPack[0] = v1Start;
      indicesToPack[1] = v1End;
      indicesToPack[2] = v2Start;
      indicesToPack[3] = v2End;
   }

   /**
    * Binary elimination helper method called if one range has a size of exactly one
    *
    * @return Array with the low and high end of each range, respectively
    */
   private static void binaryEliminationCase1(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
                                              int v2Start, int v2MedianIndex, int v2End, ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2,
                                              int[] updatedIndices)
   {
      if (v1Start == v1End)
      { // v1 contains only 1 viable vertex
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

      updatedIndices[0] = v1Start;
      updatedIndices[1] = v1End;
      updatedIndices[2] = v2Start;
      updatedIndices[3] = v2End;
   }

   private final LineSegment2D p = new LineSegment2D();

   /**
    * Binary elimination helper method called if one range has a size of exactly two
    *
    * @return Array with the low and high end of each range, respectively
    */
   private void binaryEliminationCase2(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End, int v2Start,
                                       int v2MedianIndex, int v2End, ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, int[] updatedIndices)
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
            Point2D proj = EuclidGeometryTools.orthogonalProjectionOnLine2D(polygon2.getVertex(v2MedianIndex), polygon1.getVertex(v1Start),
                                                                            polygon1.getVertex(v1End));
            p.set(polygon1.getVertex(v1Start), polygon1.getVertex(v1End));
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

      updatedIndices[0] = v1Start;
      updatedIndices[1] = v1End;
      updatedIndices[2] = v2Start;
      updatedIndices[3] = v2End;
   }

   /**
    * Binary Elimination helper method called if both ranges have size greater than two
    *
    * @return Array with the low and high end of each range, respectively
    */
   private static void binaryEliminationCase3(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
                                              int v2Start, int v2MedianIndex, int v2End, int[] updatedIndices)
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

      updatedIndices[0] = v1Start;
      updatedIndices[1] = v1End;
      updatedIndices[2] = v2Start;
      updatedIndices[3] = v2End;
   }

   /**
    * Takes in two ranges each of which are of size at most two and returns the two points on each
    * respective polygon which are closest to one another
    */
   private void getClosestPointsFromRemainingEdgesAndVertices(ConvexPolygon2DReadOnly polygon1, ConvexPolygon2DReadOnly polygon2, int v1Start, int v1End,
                                                              int v2Start, int v2End, Point2DBasics point1ToPack, Point2DBasics point2ToPack)
   {
      if ((v1Start == v1End) && (v2Start == v2End))
      {
         point1ToPack.set(polygon1.getVertex(v1Start));
         point2ToPack.set(polygon2.getVertex(v2Start));
      }
      else if (v1Start == v1End)
      {
         finalPhasePointAndEdge(polygon2.getVertex(v2Start), polygon2.getVertex(v2End), polygon1.getVertex(v1Start), point1ToPack, point2ToPack);
      }
      else if (v2Start == v2End)
      {
         finalPhasePointAndEdge(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start), point2ToPack, point1ToPack);
      }
      else
      {
         finalPhaseTwoEdges(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start), polygon2.getVertex(v2End), point1ToPack,
                            point2ToPack);
      }
   }

   private final LineSegment2D edge1 = new LineSegment2D();
   private final LineSegment2D edge2 = new LineSegment2D();
   private final Point2D proj1AOnto2 = new Point2D();
   private final Point2D proj1BOnto2 = new Point2D();
   private final Point2D proj2AOnto1 = new Point2D();
   private final Point2D proj2BOnto1 = new Point2D();
   private final Point2D[][] possiblePointPairsWithProj = new Point2D[][] {{new Point2D(), new Point2D()}, {new Point2D(), new Point2D()},
         {new Point2D(), new Point2D()}, {new Point2D(), new Point2D()}};
   private final Point2D[][] possiblePointPairsWithoutProj = new Point2D[][] {{new Point2D(), new Point2D()}, {new Point2D(), new Point2D()},
         {new Point2D(), new Point2D()}, {new Point2D(), new Point2D()}};
   private final double[] possibleDistancesWithProj = new double[4];
   private final double[] possibleDistancesWithoutProj = new double[4];
   private final boolean[] possiblePointPairsWithProjValid = new boolean[4];

   /**
    * Final phase helper method called if each range has size of exactly two
    *
    * @return The two points on each respective polygon which are closest to one another
    */
   private void finalPhaseTwoEdges(Point2DReadOnly edgePoint1A, Point2DReadOnly edgePoint1B, Point2DReadOnly edgePoint2A, Point2DReadOnly edgePoint2B,
                                   Point2DBasics point1ToPack, Point2DBasics point2ToPack)
   {
      edge1.set(edgePoint1A, edgePoint1B);
      edge2.set(edgePoint2A, edgePoint2B);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint1A, edgePoint2A, edgePoint2B, proj1AOnto2);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint1B, edgePoint2A, edgePoint2B, proj1BOnto2);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint2A, edgePoint1A, edgePoint1B, proj2AOnto1);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(edgePoint2B, edgePoint1A, edgePoint1B, proj2BOnto1);

      possiblePointPairsWithProjValid[0] = edge2.isBetweenEndpoints(proj1AOnto2);
      possiblePointPairsWithProjValid[1] = edge2.isBetweenEndpoints(proj1BOnto2);
      possiblePointPairsWithProjValid[2] = edge1.isBetweenEndpoints(proj2AOnto1);
      possiblePointPairsWithProjValid[3] = edge1.isBetweenEndpoints(proj2BOnto1);

      if (possiblePointPairsWithProjValid[0])
      {
         possiblePointPairsWithProj[0][0].set(edgePoint1A);
         possiblePointPairsWithProj[0][1].set(proj1AOnto2);
      }
      if (possiblePointPairsWithProjValid[1])
      {
         possiblePointPairsWithProj[1][0].set(edgePoint1B);
         possiblePointPairsWithProj[1][1].set(proj1BOnto2);
      }
      if (possiblePointPairsWithProjValid[2])
      {
         possiblePointPairsWithProj[2][0].set(proj2AOnto1);
         possiblePointPairsWithProj[2][1].set(edgePoint2A);
      }
      if (possiblePointPairsWithProjValid[3])
      {
         possiblePointPairsWithProj[3][0].set(proj2BOnto1);
         possiblePointPairsWithProj[3][1].set(edgePoint2B);
      }

      possiblePointPairsWithoutProj[0][0].set(edgePoint1A);
      possiblePointPairsWithoutProj[0][1].set(edgePoint2A);
      possiblePointPairsWithoutProj[1][0].set(edgePoint1A);
      possiblePointPairsWithoutProj[1][1].set(edgePoint2B);
      possiblePointPairsWithoutProj[2][0].set(edgePoint1B);
      possiblePointPairsWithoutProj[2][1].set(edgePoint2A);
      possiblePointPairsWithoutProj[3][0].set(edgePoint1B);
      possiblePointPairsWithoutProj[3][1].set(edgePoint2B);

      for (int i = 0; i < 4; i++)
      {
         possibleDistancesWithProj[i] = !possiblePointPairsWithProjValid[i] ? Double.MAX_VALUE
               : possiblePointPairsWithProj[i][0].distance(possiblePointPairsWithProj[i][1]);
         possibleDistancesWithoutProj[i] = possiblePointPairsWithoutProj[i][0].distance(possiblePointPairsWithoutProj[i][1]);
      }

      if (possibleDistancesWithProj[indexOfMin(possibleDistancesWithProj)] != Double.MAX_VALUE)
      {
         Point2D[] pair = possiblePointPairsWithProj[indexOfMin(possibleDistancesWithProj)];
         point1ToPack.set(pair[0]);
         point2ToPack.set(pair[1]);
      }
      else
      {
         Point2D[] pair = possiblePointPairsWithoutProj[indexOfMin(possibleDistancesWithoutProj)];
         point1ToPack.set(pair[0]);
         point2ToPack.set(pair[1]);
      }
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

   private final LineSegment2D pFinalPhasePoint = new LineSegment2D();

   /**
    * Final phase helper method called if one range has a size of exactly one
    *
    * @return The two points on each respective polygon which are closest to one another
    */
   private void finalPhasePointAndEdge(Point2DReadOnly edgePoint1, Point2DReadOnly edgePoint2, Point2DReadOnly lonePoint, Point2DBasics point1ToPack,
                                       Point2DBasics point2ToPack)
   {
      Point2D proj = EuclidGeometryTools.orthogonalProjectionOnLine2D(lonePoint, edgePoint1, edgePoint2);
      pFinalPhasePoint.set(edgePoint1, edgePoint2);
      if (pFinalPhasePoint.isBetweenEndpoints(proj, 0))
      {
         point1ToPack.set(lonePoint);
         point2ToPack.set(proj);
      }
      else
      {
         point1ToPack.set(lonePoint);
         point2ToPack.set((lonePoint.distance(edgePoint1) < lonePoint.distance(edgePoint2)) ? edgePoint1 : edgePoint2);
      }
   }

   /**
    * from
    * http://softsurfer.com/Archive/algorithm_0111/algorithm_0111.htm#Pseudo-Code:%20Clip%20Segment-Polygon
    * Input: a 2D segment S from point P0 to point P1 a 2D convex polygon W with n vertices
    * V0,...,Vn-1,Vn=V0
    */
   public static boolean doesSegmentIntersectConvexPolygon2D(Point2DReadOnly P0, Point2DReadOnly P1, ConvexPolygon2DReadOnly convexPolygon2d)
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

   private static boolean doesSegmentPassCompletelyThroughPolygon(Point2DReadOnly P0, Point2DReadOnly P1, ConvexPolygon2DReadOnly convexPolygon2d)
   {
      // Initialize:
      double tE = 0.0; // for the maximum entering segment parameter;
      double tL = 1.0; // for the minimum leaving segment parameter;

      // segment direction vector
      double dSx = P1.getX() - P0.getX();
      double dSy = P1.getY() - P0.getY();

      if (DEBUG)
      {
         PrintTools.info("dS = [" + dSx + ", " + dSy + "]");
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
         double V0x = convexPolygon2d.getVertexCCW(i).getX();
         double V0y = convexPolygon2d.getVertexCCW(i).getY();
         if (DEBUG)
         {
            PrintTools.info("V0 = [" + V0x + ", " + V0y + "]");
         }

         double V1x = convexPolygon2d.getNextVertexCCW(i).getX();
         double V1y = convexPolygon2d.getNextVertexCCW(i).getY();
         if (DEBUG)
         {
            PrintTools.info("V1 = [" + V1x + ", " + V1y + "]");
         }

         // edge vector
         double V0toV1x = V1x - V0x;
         double V0toV1y = V1y - V0y;

         if (DEBUG)
         {
            PrintTools.info("V0toV1 = [" + V0toV1x + ", " + V0toV1y + "]");
         }

         // outward normal of the edge
         double nix = V0toV1y;
         double niy = -V0toV1x;
         if (DEBUG)
         {
            PrintTools.info("ni = [" + nix + ", " + niy + "]");
         }

         double P0toVix = P0.getX() - V0x;
         double P0toViy = P0.getY() - V0y;

         if (DEBUG)
         {
            PrintTools.info("P0toVi = [" + P0toVix + ", " + P0toViy + "]");
         }

         double N = -(P0toVix * nix + P0toViy * niy);
         if (DEBUG)
         {
            PrintTools.info("N = " + N);
         }

         double D = dSx * nix + dSy * niy;
         if (DEBUG)
         {
            PrintTools.info("D = " + D);
         }

         if (D == 0)
         {
            // S is parallel to the edge ei

            if (N < 0)
            {
               // then P0 is outside the edge ei
               return false; // since S cannot intersect W;
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
            PrintTools.info("t = " + t);
         }

         if (D < 0)
         {
            // then segment S is entering W across edge ei
            tE = Math.max(tE, t);

            if (tE > tL)
            {
               // then segment S enters W after leaving
               return false; // since S cannot intersect W
            }
         }
         else if (D > 0)
         {
            // then segment S is leaving W across edge ei
            tL = Math.min(tL, t);

            if (tL < tE)
            {
               // then segment S leaves W before entering
               return false; // since S cannot intersect W
            }
         }
      }

      // Output: [Note: to get here, one must have tE <= tL]
      // there is a valid intersection of S with W
      // from the entering point: P(tE) = P0 + tE * dS
      // to the leaving point:    P(tL) = P0 + tL * dS
      return true;
   }

   private final ConvexPolygon2D intersectionToThrowAway = new ConvexPolygon2D();

   //TODO do something smarter here
   public double computeIntersectionAreaOfPolygons(ConvexPolygon2DReadOnly polygonP, ConvexPolygon2DReadOnly polygonQ)
   {
      if (computeIntersectionOfPolygons(polygonP, polygonQ, intersectionToThrowAway))
         return intersectionToThrowAway.getArea();
      else
         return 0.0;
   }
}
