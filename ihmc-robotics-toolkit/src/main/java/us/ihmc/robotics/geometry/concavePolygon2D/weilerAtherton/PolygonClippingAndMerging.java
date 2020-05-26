package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

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
               PolygonClippingAndMerging.merge(polygonA, polygonB, newPolygon);

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
   public static List<ConcavePolygon2DBasics> merge(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB, ConcavePolygon2DBasics mergedPolygon)
   {
      List<ConcavePolygon2DBasics> holesToReturn = new ArrayList<>();

      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonA, polygonB))
      {
         mergedPolygon.set(polygonB);
         return holesToReturn;
      }
      else if (GeometryPolygonTools.isPolygonInsideOtherPolygon(polygonB, polygonA))
      {
         mergedPolygon.set(polygonA);
         return holesToReturn;
      }

      LinkedPointList polygonAList = ClippingTools.createLinkedPointList(polygonA);
      LinkedPointList polygonBList = ClippingTools.createLinkedPointList(polygonB);

      ClippingTools.insertIntersectionsIntoList(polygonBList, polygonA);
      ClippingTools.insertIntersectionsIntoList(polygonAList, polygonB);

      Collection<LinkedPoint> unassignedPoints = polygonBList.getPointsCopy();

      LinkedPoint linkedPoint = findVertexOutsideOfPolygon(polygonA, polygonBList.getPoints());
      LinkedPointList startList = polygonBList;
      LinkedPointList otherList = polygonAList;
      if (linkedPoint == null)
      {
         linkedPoint = findVertexOutsideOfPolygon(polygonB, polygonAList.getPoints());
         startList = polygonAList;
         otherList = polygonBList;
      }

      List<LinkedPoint> pointsInPolygon = walkAlongEdgeOfPolygon(linkedPoint, startList, otherList, mergedPolygon);
      removePointsFromList(unassignedPoints, pointsInPolygon);

      polygonBList.reverseOrder();
      // FIXME this is a bad way to start. It has to start on an incoming edge
      LinkedPoint startPoint = findIntersectionPoint(unassignedPoints);
      int counter = 0;
      while (startPoint != null && counter++ < STUPID_LARGE)
      {
         ConcavePolygon2D holePolygon = new ConcavePolygon2D();
         pointsInPolygon = walkAlongEdgeOfPolygon(startPoint, polygonAList, polygonBList, holePolygon);
         holesToReturn.add(holePolygon);

         removePointsFromList(unassignedPoints, pointsInPolygon);
         startPoint = findIntersectionPoint(unassignedPoints);
      }

      return holesToReturn;
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

      // gotta make this guy counter clockwise
      clippingPolygonList.reverseOrder();

      Collection<LinkedPoint> unassignedPoints = polygonToClipList.getPointsCopy();
      LinkedPoint startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedPoints);

      int counter = 0;
      while (startPoint != null && counter++ < STUPID_LARGE)
      {
         ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
         List<LinkedPoint> pointsInPolygon = walkAlongEdgeOfPolygon(startPoint, polygonToClipList, clippingPolygonList, clippedPolygon);
         clippedPolygonsToReturn.add(clippedPolygon);

         removePointsFromList(unassignedPoints, pointsInPolygon);
         startPoint = findVertexOutsideOfPolygon(clippingPolygon, unassignedPoints);
      }

      return clippedPolygonsToReturn;
   }

   static List<LinkedPoint> walkAlongEdgeOfPolygon(LinkedPoint startPointOfPolygon,
                                                   LinkedPointList startList,
                                                   LinkedPointList otherList,
                                                   ConcavePolygon2DBasics polygonToPack)
   {
      List<LinkedPoint> pointsInPolygon = new ArrayList<>();

      LinkedPointList activeList = startList;
      LinkedPoint linkedPoint = startPointOfPolygon;
      pointsInPolygon.add(linkedPoint);

      polygonToPack.addVertex(startPointOfPolygon.getPoint());
      int counter = 0;
      while (counter++ < STUPID_LARGE)
      {
         linkedPoint = linkedPoint.getSuccessor();
         if (linkedPoint.getPoint().equals(startPointOfPolygon.getPoint()))
            break;
         pointsInPolygon.add(linkedPoint);
         polygonToPack.addVertex(linkedPoint.getPoint());

         if (linkedPoint.getIsIntersectionPoint())
         {
            // we're switching polygons
            if (activeList == startList)
               activeList = otherList;
            else
               activeList = startList;
            linkedPoint = activeList.getLinkedPointAtLocation(linkedPoint.getPoint());
            if (linkedPoint == null)
               throw new RuntimeException("Was unable to find the intersection point in the other list.");
         }
      }

      polygonToPack.update();
      return pointsInPolygon;
   }

   private static LinkedPoint findVertexOutsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (!polygon.isPointInside(point.getPoint()))
            return point;
      }

      return null;
   }

   private static LinkedPoint findIntersectionInsideOfPolygon(ConcavePolygon2DReadOnly polygon, Collection<LinkedPoint> points)
   {
      for (LinkedPoint point : points)
      {
         if (polygon.isPointInside(point.getPoint()))
            return point;
      }

      return null;
   }

   private static LinkedPoint findIntersectionPoint( Collection<LinkedPoint> polygon)
   {
      for (LinkedPoint point : polygon)
      {
         if (point.getIsIntersectionPoint())
            return point;
      }

      return null;
   }

   private static void removePointsFromList(Collection<LinkedPoint> listToEdit, Collection<LinkedPoint> pointsToRemove)
   {
      for (LinkedPoint pointToRemove : pointsToRemove)
      {
         for (LinkedPoint other : listToEdit)
         {
            if (other.getPoint().epsilonEquals(pointToRemove.getPoint(), 1e-7))
            {
               listToEdit.remove(other);
               break;
            }
         }
      }
   }
}
