package us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.concavePolygon2D.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PolygonClippingAndMerging
{
   public static void removeHolesFromList(List<ConcavePolygon2DBasics> regionsToFilter)
   {
      int i = 0;

      // first, remove all the polygons contained in another polygon
      while (i < regionsToFilter.size())
      {
         ConcavePolygon2DBasics polygonA = regionsToFilter.get(i);

         boolean shouldRemoveA = false;

         int j = 0;
         while (j < regionsToFilter.size())
         {
            if (i == j)
            {
               j++;
               continue;
            }

            ConcavePolygon2DBasics polygonB = regionsToFilter.get(j);

            if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
            {
               regionsToFilter.remove(j);
               break;
            }
            if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
            {
               shouldRemoveA = true;
               break;
            }

            j++;
         }

         if (shouldRemoveA)
            regionsToFilter.remove(i);
         else
            i++;
      }
   }

   public static void mergeAllPossible(List<ConcavePolygon2DBasics> regionsToMerge)
   {
      removeHolesFromList(regionsToMerge);

      int i = 0;
      // don't need to iterate on the last one
      while (i < regionsToMerge.size() - 1)
      {
         int j = i + 1;
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

            else
            {
               j++;
            }
         }

         i++;
      }
   }

   // TODO clean this up so that it works when the merge makes a hole.
   public static void  merge(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB, ConcavePolygon2DBasics mergedPolygon)
   {
      List<ConcavePolygon2DBasics> partialListOfHoles = new ArrayList<>();
      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
      {
         // FIXME this might have a hole
         mergedPolygon.set(polygonB);
         return;
      }
      else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
      {
         // FIXME this might have a hole
         mergedPolygon.set(polygonA);
         return;
      }
      else if (polygonA.epsilonEquals(polygonB, 1e-5))
      {
         mergedPolygon.set(polygonA);
         return;
      }

      ConcavePolygon2DClippingTools.LinkedPointList polygonAList = ConcavePolygon2DClippingTools.createLinkedPointList(polygonA);
      ConcavePolygon2DClippingTools.LinkedPointList polygonBList = ConcavePolygon2DClippingTools.createLinkedPointList(polygonB);

      ConcavePolygon2DClippingTools.insertIntersectionsIntoList(polygonAList, polygonB);
      ConcavePolygon2DClippingTools.insertIntersectionsIntoList(polygonBList, polygonA);

      ConcavePolygon2DClippingTools.linkSharedVertices(polygonAList, polygonBList, 5e-3);

      Collection<ConcavePolygon2DClippingTools.LinkedPoint> unassignedAPoints = polygonAList.getPointsCopy();
      Collection<ConcavePolygon2DClippingTools.LinkedPoint> unassignedBPoints = polygonBList.getPointsCopy();

      ConcavePolygon2DClippingTools.LinkedPointListHolder listHolder = new ConcavePolygon2DClippingTools.LinkedPointListHolder(unassignedAPoints, unassignedBPoints);

      ConcavePolygon2DClippingTools.LinkedPoint startPoint = findVertexOutsideOfPolygon(polygonB, unassignedAPoints);
      if (startPoint == null)
      {
         startPoint = findVertexOutsideOfPolygon(polygonA, unassignedBPoints);
      }

      mergedPolygon.clear();
      mergedPolygon.update();



      while (startPoint != null)
      {
         ConcavePolygon2D polygon = new ConcavePolygon2D();
         boolean failed = !walkAlongEdgeOfPolygon(startPoint, listHolder, polygon, PolygonClippingAndMerging::shouldSwitchWhenMerging);
         if (failed)
         {
            mergedPolygon.set(polygonA);
            return;
         }

         // We want the biggest of the polygons; that is, we want the outer most perimeter polygon.
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

         // check to see if there are any points outside the merged polygon on either list
         startPoint = findVertexOutsideOfPolygon(mergedPolygon, unassignedAPoints);
         if (startPoint == null)
         {
            startPoint = findVertexOutsideOfPolygon(mergedPolygon, unassignedBPoints);
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

      ConcavePolygon2DClippingTools.LinkedPointList clippingPolygonList = ConcavePolygon2DClippingTools.createLinkedPointList(clippingPolygon);
      ConcavePolygon2DClippingTools.LinkedPointList polygonToClipList = ConcavePolygon2DClippingTools.createLinkedPointList(polygonToClip);

      ConcavePolygon2DClippingTools.insertIntersectionsIntoList(polygonToClipList, clippingPolygon);
      ConcavePolygon2DClippingTools.insertIntersectionsIntoList(clippingPolygonList, polygonToClip);

      ConcavePolygon2DClippingTools.linkSharedVertices(polygonToClipList, clippingPolygonList, 5e-3);

      // gotta make the clipping list is counter clockwise
      clippingPolygonList.reverseOrder();

      Collection<ConcavePolygon2DClippingTools.LinkedPoint> unassignedToClipPoints = polygonToClipList.getPointsCopy();
      Collection<ConcavePolygon2DClippingTools.LinkedPoint> unassignedClippingPoints = clippingPolygonList.getPointsCopy();
      ConcavePolygon2DClippingTools.LinkedPointListHolder listHolder = new ConcavePolygon2DClippingTools.LinkedPointListHolder(unassignedClippingPoints, unassignedToClipPoints);

      ConcavePolygon2DClippingTools.LinkedPoint startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedToClipPoints);
      // There aren't any that are outside from the start. Instead, find a shared vertex, and make sure it's an "outgoing edge"
      if (startPoint == null)
      {
         for (ConcavePolygon2DClippingTools.LinkedPoint point : unassignedToClipPoints)
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
         ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
         boolean failed = !walkAlongEdgeOfPolygon(startPoint, listHolder, clippedPolygon, PolygonClippingAndMerging::shouldSwitchWhenClipping);
         clippedPolygonsToReturn.add(clippedPolygon);

         if (failed)
         {
            clippedPolygon.set(polygonToClip);
            return clippedPolygonsToReturn;
         }

         startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedToClipPoints);
      }

      return clippedPolygonsToReturn;
   }

   static boolean walkAlongEdgeOfPolygon(ConcavePolygon2DClippingTools.LinkedPoint startVertex, ConcavePolygon2DClippingTools.LinkedPointListHolder listHolder, ConcavePolygon2DBasics polygonToPack, SwitchingFunction switchFunction)
   {
      ConcavePolygon2DClippingTools.LinkedPoint linkedPoint = startVertex;
      ConcavePolygon2DClippingTools.LinkedPoint previousPoint = linkedPoint;

      int maxPoints = listHolder.getNumberOfPoints();
      polygonToPack.addVertex(linkedPoint.getPoint());
      boolean isOnOtherList = false;
      int counter = 0;
      while (counter < maxPoints)
      {
         linkedPoint = linkedPoint.getSuccessor();
         listHolder.removePoint(previousPoint);

         if (linkedPoint.getPoint().epsilonEquals(startVertex.getPoint(), 1e-5))
            break;

         polygonToPack.addVertex(linkedPoint.getPoint());

         boolean shouldSwitch = switchFunction.apply(linkedPoint, isOnOtherList);

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
         counter++;
      }

      if (counter >= maxPoints)
      {
         return false;
      }
//         throw new RuntimeException("Bad.");

      polygonToPack.update();
      return true;
   }


   private static boolean shouldSwitch(ConcavePolygon2DClippingTools.LinkedPoint linkedPoint)
   {
      return linkedPoint.isPointAfterInsideOther() != linkedPoint.isPointBeforeInsideOther();
   }

   @FunctionalInterface
   private interface SwitchingFunction
   {
      boolean apply(ConcavePolygon2DClippingTools.LinkedPoint point, boolean isOnOtherList);
   }

   private static boolean shouldSwitchWhenMerging(ConcavePolygon2DClippingTools.LinkedPoint linkedPoint, boolean isOnOtherList)
   {
      boolean shouldSwitch = shouldSwitch(linkedPoint);
      boolean outgoingPoint = linkedPoint.isPointBeforeInsideOther() && !linkedPoint.isPointAfterInsideOther();

      shouldSwitch |= linkedPoint.isLinkedToOtherList() && shouldSwitch(linkedPoint.getPointOnOtherList());

      if (!linkedPoint.isPointAfterInsideOther() && !linkedPoint.isPointBeforeInsideOther())
         shouldSwitch = false;
      if (outgoingPoint)
         shouldSwitch = false;

      return shouldSwitch;
   }

   private static boolean shouldSwitchWhenClipping(ConcavePolygon2DClippingTools.LinkedPoint linkedPoint, boolean isOnOtherList)
   {
      boolean shouldSwitch = shouldSwitch(linkedPoint);

      shouldSwitch |= linkedPoint.isLinkedToOtherList() && shouldSwitch(linkedPoint.getPointOnOtherList());

      if (linkedPoint.isLinkedToOtherList())
      {
         ConcavePolygon2DClippingTools.LinkedPoint clippingPoint = isOnOtherList ? linkedPoint : linkedPoint.getPointOnOtherList();
         ConcavePolygon2DClippingTools.LinkedPoint clippedPoint = isOnOtherList ? linkedPoint.getPointOnOtherList() : linkedPoint;
         if (clippingPoint.isPointBeforeInsideOther() && clippingPoint.isPointAfterInsideOther() && (!clippedPoint.isPointBeforeInsideOther()
                                                                                                     && !clippedPoint.isPointAfterInsideOther()))
         {
            shouldSwitch = true;
         }
      }

      return shouldSwitch;
   }

   private static ConcavePolygon2DClippingTools.LinkedPoint findVertexOutsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<ConcavePolygon2DClippingTools.LinkedPoint> points)
   {
      for (ConcavePolygon2DClippingTools.LinkedPoint point : points)
      {
         if (!polygon.isPointInsideEpsilon(point.getPoint(), 1e-5))
            return point;
      }

      return null;
   }

}
