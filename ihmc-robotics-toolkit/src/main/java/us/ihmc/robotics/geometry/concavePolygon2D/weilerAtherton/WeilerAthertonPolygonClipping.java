package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

public class WeilerAthertonPolygonClipping
{
   public static void clip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip, ConcavePolygon2DBasics clippedPolygon)
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
         }
      }

      clippedPolygon.update();
   }

   private static Point2DReadOnly findVertexOutsideOfClip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygon)
   {
      return polygon.getVertexBufferView().stream().filter(point -> !clippingPolygon.isPointInside(point)).findFirst().orElse(null);
   }
}
