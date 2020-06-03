package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.concavePolygon2D.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PolygonClippingAndMerging
{
   private static final int STUPID_LARGE = 1000;

   public static void mergeAllPossible(List<ConcavePolygon2DBasics> regionsToMerge)
   {
      int i = 0;
      // don't need to iterate on the last one
      while (i < regionsToMerge.size() - 1)
      {
         int j = i + 1;
         boolean shouldRemoveA = false;
         while (j < regionsToMerge.size())
         {
            ConcavePolygon2DBasics polygonA = regionsToMerge.get(i);
            ConcavePolygon2DBasics polygonB = regionsToMerge.get(j);
            if (GeometryPolygonTools.doPolygonsIntersect(polygonA, polygonB))
            {
               ConcavePolygon2D newPolygon = new ConcavePolygon2D();

               try
               {
                  PolygonClippingAndMerging.merge(polygonA, polygonB, newPolygon);
               }
               catch (ComplexPolygonException exception)
               {
                  try
                  {
                     // sometimes, because of errors, the numerics screw up and we have to try again.
                     PolygonClippingAndMerging.merge(polygonA, polygonB, newPolygon);
                  }
                  catch (ComplexPolygonException repeatException)
                  {
                     j++;
                     LogTools.info("Caught an error when trying to merge.");
                     continue;
                  }
               }

               regionsToMerge.set(i, newPolygon);
               regionsToMerge.remove(j);

               // reset the search, as we modified the first polygon
               j = i + 1;
            }
            else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
            {
               regionsToMerge.remove(j);
            }
            else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
            {
               shouldRemoveA = true;
               break;
            }
            else
            {
               j++;
            }
         }

         if (shouldRemoveA)
            regionsToMerge.remove(i);
         else
            i++;
      }
   }

   public static void merge(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB, ConcavePolygon2DBasics mergedPolygon)
   {
      List<ConcavePolygon2D> partialListOfHoles = new ArrayList<>();
      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
      {
         mergedPolygon.set(polygonB);
         return;
      }
      else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
      {
         mergedPolygon.set(polygonA);
         return;
      }

      LinkedPointList polygonAList = ClippingTools.createLinkedPointList(polygonA);
      LinkedPointList polygonBList = ClippingTools.createLinkedPointList(polygonB);

      ClippingTools.insertIntersectionsIntoList(polygonAList, polygonB);
      ClippingTools.insertIntersectionsIntoList(polygonBList, polygonA);
      ClippingTools.linkSharedVertices(polygonAList, polygonBList);

      Collection<LinkedPoint> unassignedAPoints = polygonAList.getPointsCopy();
      Collection<LinkedPoint> unassignedBPoints = polygonBList.getPointsCopy();

      LinkedPointProvider pointProvider = new LinkedPointProvider(polygonAList, polygonBList, unassignedAPoints, unassignedBPoints);

      boolean startOnListA = true;
      LinkedPoint startPoint = findVertexOutsideOfPolygon(polygonB, unassignedAPoints);
      if (startPoint == null)
      {
         startPoint = findVertexOutsideOfPolygon(polygonA, unassignedBPoints);
         startOnListA = false;
      }

      mergedPolygon.clear();
      mergedPolygon.update();

      while (startPoint != null)
      {
         pointProvider.setStart(startPoint, startOnListA);

         ConcavePolygon2D polygon = new ConcavePolygon2D();
         walkAlongEdgeOfPolygon(pointProvider, polygon, false);

         if (Double.isNaN(mergedPolygon.getArea()))
         {
            mergedPolygon.set(polygon);
         }
         else if (polygon.getArea() > mergedPolygon.getArea())
         {
            partialListOfHoles.add(new ConcavePolygon2D(mergedPolygon));
            mergedPolygon.set(polygon);
         }
         else
         {
            partialListOfHoles.add(polygon);
         }

         startPoint = findVertexOutsideOfPolygon(mergedPolygon, unassignedAPoints);
         startOnListA = true;
         if (startPoint == null)
         {
            findVertexOutsideOfPolygon(mergedPolygon, unassignedBPoints);
            startOnListA = false;
         }
      }
   }

   public static List<ConcavePolygon2DBasics> removeAreaInsideClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip)
   {
      List<ConcavePolygon2DBasics> clippedPolygonsToReturn = new ArrayList<>();
      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(clippingPolygon, polygonToClip))
      {
         ConcavePolygon2D polygonToReturn = new ConcavePolygon2D(clippingPolygon);
         clippedPolygonsToReturn.add(polygonToReturn);

         return clippedPolygonsToReturn;
      }
      else if (!GeometryPolygonTools.doPolygonsIntersect(clippingPolygon, polygonToClip))
      {
         ConcavePolygon2D polygonToReturn = new ConcavePolygon2D(polygonToClip);
         clippedPolygonsToReturn.add(polygonToReturn);

         return clippedPolygonsToReturn;
      }

      LinkedPointList clippingPolygonList = ClippingTools.createLinkedPointList(clippingPolygon);
      LinkedPointList polygonToClipList = ClippingTools.createLinkedPointList(polygonToClip);

      ClippingTools.insertIntersectionsIntoList(polygonToClipList, clippingPolygon);
      ClippingTools.insertIntersectionsIntoList(clippingPolygonList, polygonToClip);
      ClippingTools.linkSharedVertices(polygonToClipList, clippingPolygonList);

      // gotta make this guy counter clockwise
      clippingPolygonList.reverseOrder();

      Collection<LinkedPoint> unassignedToClipPoints = polygonToClipList.getPointsCopy();
      Collection<LinkedPoint> unassignedClippingPoints = clippingPolygonList.getPointsCopy();
      LinkedPointProvider pointProvider = new LinkedPointProvider(clippingPolygonList, polygonToClipList, unassignedClippingPoints, unassignedToClipPoints);

      LinkedPoint startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedToClipPoints);
      if (startPoint == null)
      {
         for (LinkedPoint point : unassignedToClipPoints)
         {
            if (!point.isPointAfterInsideOther() && point.isPointBeforeInsideOther())
            {
               startPoint = point;
               break;
            }
         }
      }

      while (startPoint != null)
      {
         pointProvider.setStart(startPoint, false);

         ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
         walkAlongEdgeOfPolygon(pointProvider, clippedPolygon, true);
         clippedPolygonsToReturn.add(clippedPolygon);

         startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedToClipPoints);
      }

      if (clippedPolygonsToReturn.size() < 1)
      {
         LogTools.info("WHat?");
         GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonToClip, clippingPolygon);
         startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedToClipPoints);
      }
      return clippedPolygonsToReturn;
   }

   static void walkAlongEdgeOfPolygon(LinkedPointProvider pointProvider, ConcavePolygon2DBasics polygonToPack, boolean isClipping)
   {
      LinkedPoint linkedPoint = pointProvider.getCurrentPoint();
      LinkedPoint previousPoint = linkedPoint;

      polygonToPack.addVertex(linkedPoint.getPoint());
      int counter = 0;
      boolean isOnOtherList = false;
      while (counter++ < STUPID_LARGE)
      {
         linkedPoint = linkedPoint.getSuccessor();
         pointProvider.removePoint(previousPoint);

         if (linkedPoint.getPoint().epsilonEquals(pointProvider.getStartVertex().getPoint(), 1e-7))
            break;

         polygonToPack.addVertex(linkedPoint.getPoint());

         boolean shouldSwitch = shouldSwitch(linkedPoint);
         boolean outgoingPoint = linkedPoint.isPointBeforeInsideOther() && !linkedPoint.isPointAfterInsideOther();

         shouldSwitch |= linkedPoint.isLinkedToOtherList() && shouldSwitch(linkedPoint.getPointOnOtherList());

         if (isClipping && linkedPoint.isLinkedToOtherList())
         {
            LinkedPoint clippingPoint = isOnOtherList ? linkedPoint : linkedPoint.getPointOnOtherList();
            LinkedPoint clippedPoint = isOnOtherList ? linkedPoint.getPointOnOtherList() : linkedPoint;
            if (clippingPoint.isPointBeforeInsideOther() && clippingPoint.isPointAfterInsideOther() && (!clippedPoint.isPointBeforeInsideOther()
                                                                                                        && !clippedPoint.isPointAfterInsideOther()))
            {
               shouldSwitch = true;
            }
         }
         if (!isClipping && !linkedPoint.isPointAfterInsideOther() && !linkedPoint.isPointBeforeInsideOther())
            shouldSwitch = false;
         if (!isClipping && outgoingPoint)
            shouldSwitch = false;

         if (shouldSwitch)
         {
            // we're switching polygons
            isOnOtherList = !isOnOtherList;
            previousPoint = linkedPoint;
            linkedPoint = linkedPoint.getPointOnOtherList();

            if (linkedPoint == null)
               throw new RuntimeException("Was unable to find the intersection point in the other list.");
         }
         else
         {
            previousPoint = linkedPoint;
         }
      }

      polygonToPack.update();
   }

   private static boolean shouldSwitch(LinkedPoint linkedPoint)
   {
      return linkedPoint.isPointAfterInsideOther() != linkedPoint.isPointBeforeInsideOther();
   }

   private static LinkedPoint findVertexOutsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (!polygon.isPointInsideEpsilon(point.getPoint(), 1e-5))
            return point;
      }

      return null;
   }

   private static LinkedPoint findVertexOnPerimeterOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (GeometryPolygonTools.isPoint2DOnPerimeterOfSimplePolygon2D(point.getPoint(), polygon.getVertexBufferView(), polygon.getNumberOfVertices(), 1e-5))
            return point;
      }

      return null;
   }
}
