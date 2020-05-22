package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PolygonClippingAndMerging
{
   public static void merge(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB, ConcavePolygon2DBasics mergedPolygon)
   {
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

      ClippingTools.insertIntersectionsIntoList(polygonBList, polygonA);
      ClippingTools.insertIntersectionsIntoList(polygonAList, polygonB);

      LinkedPoint linkedPoint = findVertexOutsideOfClip(polygonA, polygonBList.getPoints());

      walkAlongEdgeOfPolygon(linkedPoint, polygonBList, polygonAList, mergedPolygon);
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
      LinkedPoint startPoint = findVertexOutsideOfClip(clippingPolygon, unassignedPoints);

      while (startPoint != null)
      {
         ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
         List<LinkedPoint> pointsInPolygon = walkAlongEdgeOfPolygon(startPoint, polygonToClipList, clippingPolygonList, clippedPolygon);
         clippedPolygonsToReturn.add(clippedPolygon);

         removePointsFromList(unassignedPoints, pointsInPolygon);
         startPoint = findVertexOutsideOfClip(clippingPolygon, unassignedPoints);
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
      while (true)
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

   private static Point2DReadOnly findVertexOutsideOfClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygon)
   {
      return polygon.getVertexBufferView().stream().filter(point -> !clippingPolygon.isPointInside(point)).findFirst().orElse(null);
   }

   private static LinkedPoint findVertexOutsideOfClip(ConcavePolygon2DReadOnly clippingPolygon, Collection<LinkedPoint> polygon)
   {
      for (LinkedPoint point : polygon)
      {
         if (!clippingPolygon.isPointInside(point.getPoint()))
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
