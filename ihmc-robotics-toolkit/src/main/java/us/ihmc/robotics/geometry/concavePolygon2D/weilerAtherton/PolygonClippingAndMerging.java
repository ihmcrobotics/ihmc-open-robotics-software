package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

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

      LinkedPointList clippingPolygonList = ClippingTools.createLinkedPointList(polygonA);
      LinkedPointList polygonToClipList = ClippingTools.createLinkedPointList(polygonB);

      ClippingTools.insertIntersectionsIntoList(polygonToClipList, polygonA);
      ClippingTools.insertIntersectionsIntoList(clippingPolygonList, polygonB);

      Point2DReadOnly startVertex = findVertexOutsideOfClip(polygonA, polygonB);
      LinkedPointList activeList = polygonToClipList;
      LinkedPoint linkedPoint = activeList.getLinkedPointAtLocation(startVertex);

      mergedPolygon.clear();
      mergedPolygon.addVertex(startVertex);
      while (true)
      {
         linkedPoint = linkedPoint.getSuccessor();
         if (linkedPoint.getPoint().equals(startVertex))
            break;
         mergedPolygon.addVertex(linkedPoint.getPoint());

         if (linkedPoint.getIsIntersectionPoint())
         {
            // we're switching polygons
            if (activeList == polygonToClipList)
               activeList = clippingPolygonList;
            else
               activeList = polygonToClipList;
            linkedPoint = activeList.getLinkedPointAtLocation(linkedPoint.getPoint());
         }
      }

      mergedPolygon.update();
   }

   public static void removeAreaInsideClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip, ConcavePolygon2DBasics clippedPolygon)
   {
      if (GeometryPolygonTools.isPolygonInsideOtherPolygon(clippingPolygon, polygonToClip))
      {
         clippedPolygon.set(clippingPolygon);
         return;
      }
      else if (!GeometryPolygonTools.doPolygonsIntersect(clippingPolygon, polygonToClip))
      {
         clippedPolygon.set(polygonToClip);
         return;
      }

      LinkedPointList clippingPolygonList = ClippingTools.createLinkedPointList(clippingPolygon);
      LinkedPointList polygonToClipList = ClippingTools.createLinkedPointList(polygonToClip);

      ClippingTools.insertIntersectionsIntoList(polygonToClipList, clippingPolygon);
      ClippingTools.insertIntersectionsIntoList(clippingPolygonList, polygonToClip);

      // gotta make this guy counter clockwise
      clippingPolygonList.reverseOrder();

      Point2DReadOnly startVertex = findVertexOutsideOfClip(clippingPolygon, polygonToClip);
      LinkedPointList activeList = polygonToClipList;
      LinkedPoint linkedPoint = activeList.getLinkedPointAtLocation(startVertex);

      clippedPolygon.clear();
      clippedPolygon.addVertex(startVertex);
      while (true)
      {
         linkedPoint = linkedPoint.getSuccessor();
         if (linkedPoint.getPoint().equals(startVertex))
            break;
         clippedPolygon.addVertex(linkedPoint.getPoint());

         if (linkedPoint.getIsIntersectionPoint())
         {
            // we're switching polygons
            if (activeList == polygonToClipList)
               activeList = clippingPolygonList;
            else
               activeList = polygonToClipList;
            linkedPoint = activeList.getLinkedPointAtLocation(linkedPoint.getPoint());
            if (linkedPoint == null)
               throw new RuntimeException("Was unable to find the intersection point in the other list.");
         }
      }

      clippedPolygon.update();
   }

   private static Point2DReadOnly findVertexOutsideOfClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygon)
   {
      return polygon.getVertexBufferView().stream().filter(point -> !clippingPolygon.isPointInside(point)).findFirst().orElse(null);
   }
}
