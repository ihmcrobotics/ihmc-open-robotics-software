package us.ihmc.robotics.geometry;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.ArrayList;

public class ConvexPolygonToolbox
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
   public static boolean findConnectingEdgesVerticesIndexes(ConvexPolygon2D polygon1, ConvexPolygon2D polygon2, VerticesIndices polygon1VerticesIndicesToPack,
         VerticesIndices polygon2VerticesIndicesToPack)
   {
      boolean success = false;

      polygon1VerticesIndicesToPack.setNumberOfIndices(2);
      polygon2VerticesIndicesToPack.setNumberOfIndices(2);

      if (polygon1.isEmpty() || polygon2.isEmpty())
         return false;

      if (polygon1.getNumberOfVertices() == 1 && polygon2.getNumberOfVertices() == 1)
      {
         polygon1VerticesIndicesToPack.setIndex(0, 0);
         polygon1VerticesIndicesToPack.setIndex(1, 0);
         polygon2VerticesIndicesToPack.setIndex(0, 0);
         polygon2VerticesIndicesToPack.setIndex(1, 0);
         return true;
      }

      if (polygon1.getNumberOfVertices() == 1)
      {
         polygon1VerticesIndicesToPack.setIndex(0, 0);
         polygon1VerticesIndicesToPack.setIndex(1, 0);
         int lineOfSightStartIndex = polygon2.lineOfSightStartIndex(polygon1.getVertex(0));
         int lineOfSightEndIndex = polygon2.lineOfSightEndIndex(polygon1.getVertex(0));
         success = lineOfSightStartIndex > -1 && lineOfSightEndIndex > -1;
         polygon2VerticesIndicesToPack.setIndex(0, lineOfSightStartIndex);
         polygon2VerticesIndicesToPack.setIndex(1, lineOfSightStartIndex);
         return success;
      }

      if (polygon2.getNumberOfVertices() == 1)
      {
         polygon2VerticesIndicesToPack.setIndex(0, 0);
         polygon2VerticesIndicesToPack.setIndex(1, 0);

         int lineOfSightStartIndex = polygon1.lineOfSightStartIndex(polygon2.getVertex(0));
         int lineOfSightEndIndex = polygon1.lineOfSightEndIndex(polygon2.getVertex(0));
         success = lineOfSightStartIndex > -1 && lineOfSightEndIndex > -1;
         polygon1VerticesIndicesToPack.setIndex(0, lineOfSightStartIndex);
         polygon1VerticesIndicesToPack.setIndex(1, lineOfSightStartIndex);
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

      polygon1VerticesIndicesToPack.setIndex(0, L1);
      polygon1VerticesIndicesToPack.setIndex(1, R1);
      polygon2VerticesIndicesToPack.setIndex(0, R2);
      polygon2VerticesIndicesToPack.setIndex(1, L2);
      return true;
   }

   private final VerticesIndices polygon1VerticesIndices = new VerticesIndices();
   private final VerticesIndices polygon2VerticesIndices = new VerticesIndices();
   /**
    * Efficiently combines two Disjoint Polygons. Returns false if not disjoint.
    *
    * @param polygon1 ConvexPolygon2d
    * @param polygon2 ConvexPolygon2d
    * @param combinedPolygonToPack ConvexPolygon2d polygon in which we put the convex hull
    *           containing polygon1 and polygon2.
    * @param connectingEdge1ToPack LineSegment2d first connecting edge between polygon1 and
    *           polygon2.
    * @param connectingEdge2ToPack LineSegment2d second connecting edge between polygon1 and
    *           polygon2.
    * @return true if succeeded, false if failed
    */
   public boolean combineDisjointPolygons(ConvexPolygon2D polygon1, ConvexPolygon2D polygon2, ConvexPolygon2D combinedPolygonToPack,
                                                 LineSegment2D connectingEdge1ToPack, LineSegment2D connectingEdge2ToPack)
   {
      boolean success = findConnectingEdgesVerticesIndexes(polygon1, polygon2, polygon1VerticesIndices, polygon2VerticesIndices);
      if (!success)
         return false;

      combinedPolygonToPack.clear();
      polygon1.getVerticesInClockwiseOrder(polygon1VerticesIndices.getIndex(1), polygon1VerticesIndices.getIndex(0), combinedPolygonToPack);
      polygon2.getVerticesInClockwiseOrder(polygon2VerticesIndices.getIndex(0), polygon2VerticesIndices.getIndex(1), combinedPolygonToPack);
      combinedPolygonToPack.update();

      getConnectingEdges(polygon1, polygon2, connectingEdge1ToPack, connectingEdge2ToPack, polygon1VerticesIndices, polygon2VerticesIndices);

      return true;
   }


   static void getConnectingEdges(ConvexPolygon2D polygon1, ConvexPolygon2D polygon2, LineSegment2D connectingEdge1ToPack,
                                          LineSegment2D connectingEdge2ToPack, VerticesIndices polygon1VerticesIndices,
                                          VerticesIndices polygon2VerticesIndices)
   {
      connectingEdge1ToPack.set(polygon1.getVertex(polygon1VerticesIndices.getIndex(0)), polygon2.getVertex(polygon2VerticesIndices.getIndex(0)));
      connectingEdge2ToPack.set(polygon2.getVertex(polygon2VerticesIndices.getIndex(1)), polygon1.getVertex(polygon1VerticesIndices.getIndex(1)));
   }


   private final ConvexPolygon2D combinedPolygon = new ConvexPolygon2D();
   private final LineSegment2D connectingEdge1 = new LineSegment2D();
   private final LineSegment2D connectingEdge2 = new LineSegment2D();
   /**
    * Efficiently combines two Disjoint Polygons. Returns null if not disjoint. Note: Generates
    * garbage!
    *
    * @param polygon1 ConvexPolygon2d
    * @param polygon2 ConvexPolygon2d
    * @return ConvexPolygon2dAndConnectingEdges
    */
   public boolean combineDisjointPolygons(ConvexPolygon2D polygon1, ConvexPolygon2D polygon2, ConvexPolygon2dAndConnectingEdges combinedPolygonAndEdgesToPack)
   {
      if (!combineDisjointPolygons(polygon1, polygon2, combinedPolygon, connectingEdge1, connectingEdge2))
         return false;

      combinedPolygonAndEdgesToPack.set(combinedPolygon, connectingEdge1, connectingEdge2);

      return true;
   }

   private final Vector2D caliperForPolygonP = new Vector2D();
   private final Vector2D caliperForPolygonQ = new Vector2D();
   private final Point2D lineStart = new Point2D();
   private final Point2D lineEnd = new Point2D();

   private final Vector2D vectorToNextPointOnPolygonP = new Vector2D();
   private final Vector2D vectorToNextPointOnPolygonQ = new Vector2D();

   private static final int initialCapacity = 10;
   private final RecyclingArrayList<MutableInt> bridgeIndicesP = new RecyclingArrayList<>(initialCapacity, MutableInt.class);
   private final RecyclingArrayList<MutableInt> bridgeIndicesQ = new RecyclingArrayList<>(initialCapacity, MutableInt.class);
   private final RecyclingArrayList<MutableBoolean> bridgeWasOnLeft = new RecyclingArrayList<>(initialCapacity, MutableBoolean.class);

   /**
    * Computes the intersection of two convex polygons. For references see:
    * http://www.iro.umontreal.ca/~plante/compGeom/algorithm.html Returns null if the polygons do
    * not intersect Returns the inside polygon if the two polygons are inside one another.
    *
    * @param polygonP ConvexPolygon2d
    * @param polygonQ ConvexPolygon2d
    * @return ConvexPolygon2d Intersection of polygonP and polygonQ
    */
   public boolean computeIntersectionOfPolygons(ConvexPolygon2D polygonP, ConvexPolygon2D polygonQ, ConvexPolygon2D intersectingPolygonToPack)
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
      int currentPPolygonPointIndex = polygonP.getMinXIndex();
      int initialPolygonPIndex = polygonP.getMinXIndex();
      Point2DReadOnly currentPolygonPPoint = polygonP.getVertex(currentPPolygonPointIndex);

      // Find left most point on polygon2
      int currentQPolygonPointIndex = polygonQ.getMinXIndex();
      int initialPolygonQIndex = polygonQ.getMinXIndex();
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

      boolean gotAroundPOnce = false;
      boolean gotAroundQOnce = false;
      boolean DONE = false;

      bridgeIndicesP.clear();
      bridgeIndicesQ.clear();
      bridgeWasOnLeft.clear();

      int bridgeCount = 0;

      do
      {
         if (gotAroundPOnce && gotAroundQOnce)
         {
            DONE = true;
         }

         // Rotate these two lines (called calipers) by the smallest angle between a caliper and the segment following the vertex it passes
         // through (in clockwise order). The rotation is done about the vertex through which the line passes on the associated polygon.
         // If the line passes through more than one vertex of the associated polygon, the farthest (in clockwise order) is taken.

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

         if (dotProductToNextPointOnPolygonP == dotProductToNextPointOnPolygonQ)
         {
            caliperForPolygonP.set(vectorToNextPointOnPolygonP);
            caliperForPolygonQ.set(caliperForPolygonP);

            moveCaliperP = true;
            moveCaliperQ = true;
         }

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

         // determine which side polygon Q's caliper line is on relative to polygon P's caliper lline
         lineStart.set(currentPolygonPPoint);
         lineEnd.set(currentPolygonPPoint);
         lineEnd.add(caliperForPolygonP);
         isOnLeft = EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(currentPolygonQPoint, lineStart, lineEnd);


         if (wasOnLeft != isOnLeft)
         {
            // find all of the bridges

            // Some weird fence post thing here. Sometime you want to consider the start ones at the end, sometimes you don't.
            // So that's why DONE is computed at the top and checked on the bottom...

            boolean addThisOne = true;
            if ((DONE) && (!bridgeIndicesP.isEmpty()))
            {
               if ((bridgeIndicesP.get(0).intValue() == currentPPolygonPointIndex) && (bridgeIndicesQ.get(0).intValue() == currentQPolygonPointIndex))
               {
                  addThisOne = false;
               }
            }

            if (addThisOne)
            {
               bridgeIndicesP.add().setValue(currentPPolygonPointIndex);
               bridgeIndicesQ.add().setValue(currentQPolygonPointIndex);
               bridgeWasOnLeft.add().setValue(wasOnLeft);

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

   private static boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyOneVertex(ConvexPolygon2D polygonWithExactlyOneVertex,
                                                                                       ConvexPolygon2D otherPolygon, ConvexPolygon2D intersectingPolygon)
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

   private final LineSegment2D polygonWithTwoVerticesAsLineSegment = new LineSegment2D();

   private boolean computeIntersectionOfPolygonsIfOnePolygonHasExactlyTwoVerticesAndTheOtherHasAtLeastTwoVertices(ConvexPolygon2D polygonWithExactlyTwoVertices,
                                                                                                                  ConvexPolygon2D polygonWithAtLeastTwoVertices,
                                                                                                                  ConvexPolygon2D intersectingPolygon)
   {
      polygonWithTwoVerticesAsLineSegment.set(polygonWithExactlyTwoVertices.getVertex(0), polygonWithExactlyTwoVertices.getVertex(1));
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

   private static boolean findCrossingIndices(StartAndEndIndices crossingIndices, boolean decrementP, int bridgeIndexForPolygonP, int bridgeIndexForPolygonQ,
         ConvexPolygon2D polygonP, ConvexPolygon2D polygonQ)
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

      crossingIndices.setIndex1Start(indexPStart);
      crossingIndices.setIndex1End(indexPEnd);
      crossingIndices.setIndex2Start(indexQStart);
      crossingIndices.setIndex2End(indexQEnd);

      return true;
   }

   static boolean constructPolygonForIntersection(RecyclingArrayList<StartAndEndIndices> crossingIndicesList, ConvexPolygon2D polygonP,
                                                  ConvexPolygon2D polygonQ, ConvexPolygon2D intersectingPolygonToPack)
   {
      StartAndEndIndices crossingIndices = crossingIndicesList.getFirst();
      int startIndexP1 = crossingIndices.getIndex1Start();
      int endIndexP1 = crossingIndices.getIndex1End();

      int startIndexQ1 = crossingIndices.getIndex2Start();
      int endIndexQ1 = crossingIndices.getIndex2End();

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
      for (int i = 0; i < crossingIndicesList.size(); i++)
      {
         crossingIndices = crossingIndicesList.get(i);
         int startIndexP = crossingIndices.getIndex1Start();
         int endIndexP = crossingIndices.getIndex1End();

         int startIndexQ = crossingIndices.getIndex2Start();
         int endIndexQ = crossingIndices.getIndex2End();

         Point2DReadOnly startP = polygonP.getVertex(startIndexP);
         Point2DReadOnly endP = polygonP.getVertex(endIndexP);
         Point2DReadOnly startQ = polygonQ.getVertex(startIndexQ);
         Point2DReadOnly endQ = polygonQ.getVertex(endIndexQ);

         Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(startP, endP, startQ, endQ);
         if (intersection == null)
         {
            System.err.println("intersection is null in constructPolygonForIntersection!. startP = " + startP + ", endP = " + endP + ", startQ = " + startQ
                  + ", endQ = " + endQ);
            System.err.println("startIndexP = " + startIndexP + ", endIndexP = " + endIndexP);
            System.err.println("Returning null polygon");

            intersectingPolygonToPack.clear();
            intersectingPolygonToPack.update();
            return false;
         }

         intersectingPolygonToPack.addVertex(intersection);

         if (incrementPNext)
         {
            int indexP = endIndexP;
            int indexPNext = crossingIndicesList.get((i + 1) % crossingIndicesList.size()).getIndex1Start();

            while (indexP != indexPNext)
            {
               intersectingPolygonToPack.addVertex(polygonP.getVertex(indexP));
               indexP = polygonP.getNextVertexIndex(indexP);
            }
         }
         else
         {
            int indexQ = endIndexQ;
            int indexQNext = crossingIndicesList.get((i + 1) % crossingIndicesList.size()).getIndex2Start();

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

   private final RecyclingArrayList<StartAndEndIndices> crossingIndices = new RecyclingArrayList<>(initialCapacity, new StartAndEndIndicesBuilder());

   private boolean buildCommonPolygonFromBridges(RecyclingArrayList<MutableInt> bridgeIndicesP, RecyclingArrayList<MutableInt> bridgeIndicesQ,
         RecyclingArrayList<MutableBoolean> bridgeWasOnLeft, ConvexPolygon2D polygonP, ConvexPolygon2D polygonQ, ConvexPolygon2D intersectingPolygonToPack)
   {
      crossingIndices.clear();

      for (int i = 0; i < bridgeIndicesP.size(); i++)
      {
         // find intersection for bridge
         int bridgeIndexForPolygonP = bridgeIndicesP.get(i).getValue();
         int bridgeIndexForPolygonQ = bridgeIndicesQ.get(i).getValue();

         // for each bridge, compute the intersection points

         if (!findCrossingIndices(crossingIndices.add(), bridgeWasOnLeft.get(i).getValue(), bridgeIndexForPolygonP, bridgeIndexForPolygonQ, polygonP, polygonQ))
         {
            intersectingPolygonToPack.clearAndUpdate();
            return false; // No intersection.
         }
      }

      return constructPolygonForIntersection(crossingIndices, polygonP, polygonQ, intersectingPolygonToPack);
   }


   private final ConvexPolygon2dAndConnectingEdges combinedPolygonAndEdges = new ConvexPolygon2dAndConnectingEdges();
   /**
    * @param polygon1
    * @param polygon2
    * @return success
    */
   //this should throw away points that are inside of the other polygon
   public boolean combineDisjointPolygons(FrameConvexPolygon2d polygon1, FrameConvexPolygon2d polygon2,
         FrameConvexPolygon2dAndConnectingEdges combinedPolygonToPack)
   {
      ReferenceFrame referenceFrame = polygon1.getReferenceFrame();
      polygon2.checkReferenceFrameMatch(referenceFrame);

      if (!combineDisjointPolygons(polygon1.convexPolygon, polygon2.convexPolygon, combinedPolygonAndEdges))
         return false;// Return false if not disjoint

      combinedPolygonToPack.setIncludingFrameAndUpdate(referenceFrame, polygon1, polygon2, combinedPolygonAndEdges.getConvexPolygon2d(), combinedPolygonAndEdges.getConnectingEdge1(),
            combinedPolygonAndEdges.getConnectingEdge2());

      return true;
   }


   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It
    * is conservative in the sense, that the modified polygon will be contained in the original
    * polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(ConvexPolygon2D polygon, int desiredVertices)
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

   }

   /**
    * This function changes the polygon given, such that it has the desired number of vertices. It
    * is conservative in the sense, that the modified polygon will be contained in the original
    * polygon completely.
    *
    * @param polygon: modified to have the desired number of vertices
    * @param desiredVertices: number of vertices that the polygon should have
    */
   public static void limitVerticesConservative(FrameConvexPolygon2d polygon, int desiredVertices)
   {
      limitVerticesConservative(polygon.getConvexPolygon2d(), desiredVertices);
   }


   public class StartAndEndIndicesBuilder extends GenericTypeBuilder<StartAndEndIndices>
   {
      @Override
      public StartAndEndIndices newInstance()
      {
         return new StartAndEndIndices();
      }
   }

   VerticesIndices createVerticesIndices()
   {
      return new VerticesIndices();
   }

   StartAndEndIndicesBuilder createStartAndEndIndicesBuilder()
   {
      return new StartAndEndIndicesBuilder();
   }


   class StartAndEndIndices
   {
      private int index1Start;
      private int index1End;
      private int index2Start;
      private int index2End;

      public void setIndex1Start(int index1Start)
      {
         this.index1Start = index1Start;
      }

      public void setIndex2Start(int index2Start)
      {
         this.index2Start = index2Start;
      }

      public void setIndex1End(int index1End)
      {
         this.index1End = index1End;
      }

      public void setIndex2End(int index2End)
      {
         this.index2End = index2End;
      }

      public int getIndex1Start()
      {
         return index1Start;
      }

      public int getIndex1End()
      {
         return index1End;
      }

      public int getIndex2Start()
      {
         return index2Start;
      }

      public int getIndex2End()
      {
         return index2End;
      }
   }

   class VerticesIndices
   {
      private final RecyclingArrayList<MutableInt> connectingEdgeIndices = new RecyclingArrayList<MutableInt>(20, MutableInt.class);

      public void clear()
      {
         connectingEdgeIndices.clear();
      }

      public int size()
      {
         return connectingEdgeIndices.size();
      }

      public void setNumberOfIndices(int numberOfIndices)
      {
         clear();
         for (int i = 0; i < numberOfIndices; i++)
            connectingEdgeIndices.add();
      }

      public void addIndex(int index)
      {
         connectingEdgeIndices.add().setValue(index);
      }

      public void setIndex(int number, int index)
      {
         connectingEdgeIndices.get(number).setValue(index);
      }

      public int getIndex(int number)
      {
         return connectingEdgeIndices.get(number).getValue();
      }
   }
}
