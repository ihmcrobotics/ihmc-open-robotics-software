package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class WeilerAthertonPolygonClipping
{
   public static void clip(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip, ConcavePolygon2DBasics clippedPolygon)
   {
      if (!GeometryPolygonTools.doPolygonsIntersect(clippingPolygon, polygonToClip))
         throw new IllegalArgumentException("Polygons don't intersect.");

      clippedPolygon.clear();

      int startIdx = findPointThatWontBeClipped(clippingPolygon, polygonToClip);
      clippedPolygon.addVertex(polygonToClip.getVertex(startIdx));

      boolean needsInitializing = true;
      int vertexIdx = startIdx;
      while (vertexIdx != startIdx || needsInitializing)
      {
         int nextIdx = EuclidGeometryPolygonTools.next(vertexIdx, polygonToClip.getNumberOfVertices());
         needsInitializing = false;

         Point2DReadOnly nextVertex = polygonToClip.getVertex(nextIdx);
         boolean nextPointIsInside = clippingPolygon.isPointInside(nextVertex);

         // handle the clipping loop
         if (nextPointIsInside)
         {
            Point2DReadOnly pointOutside = polygonToClip.getVertex(vertexIdx);
            Point2D intersection = new Point2D();
            int clippingIdx = findIntersection(clippingPolygon, pointOutside, nextVertex, intersection);

            if (clippingIdx == -1)
               throw new RuntimeException("Failed to find an intersection.");

            clippedPolygon.addVertex(intersection);
            clippedPolygon.addVertex(polygonToClip.getVertex(clippingIdx));

            clippingIdx = EuclidGeometryPolygonTools.previous(clippingIdx, clippingPolygon.getNumberOfVertices());
            while (polygonToClip.isPointInside(clippingPolygon.getVertex(clippingIdx)))
               clippingIdx = EuclidGeometryPolygonTools.previous(clippingIdx, clippingPolygon.getNumberOfVertices());

            vertexIdx = findIntersection(polygonToClip, clippedPolygon.getVertex(clippingIdx), clippedPolygon.getVertex(EuclidGeometryPolygonTools.next(clippingIdx, clippingPolygon.getNumberOfVertices())), intersection);
            vertexIdx = EuclidGeometryPolygonTools.next(vertexIdx, polygonToClip.getNumberOfVertices());
            clippedPolygon.addVertex(intersection);
         }
         else
         {
            vertexIdx = nextIdx;
         }
      }

   }

   private static int findPointThatWontBeClipped(ConcavePolygon2DReadOnly clippingPolygon, ConcavePolygon2DReadOnly polygonToClip)
   {
      int outsideIndex = 0;
      while (clippingPolygon.isPointInside(polygonToClip.getVertex(outsideIndex)))
         outsideIndex = EuclidGeometryPolygonTools.previous(outsideIndex, polygonToClip.getNumberOfVertices());

      return outsideIndex;
   }

   private static int findIntersection(Vertex2DSupplier polygon, Point2DReadOnly pointOutside, Point2DReadOnly pointInside, Point2DBasics intersectionToPack)
   {
      return findIntersection(polygon, 0, pointOutside, pointInside, intersectionToPack);
   }

   private static int findIntersection(Vertex2DSupplier polygon, int startIndex, Point2DReadOnly pointOutside, Point2DReadOnly pointInside, Point2DBasics intersectionToPack)
   {
      intersectionToPack.setToNaN();

      int vertexIdx = startIndex;
      boolean needsInitializing = true;

      while (vertexIdx != startIndex || needsInitializing)
      {
         needsInitializing = false;
         int nextIdx = EuclidGeometryPolygonTools.next(vertexIdx, polygon.getNumberOfVertices());
         Point2DReadOnly startVertex = polygon.getVertex(vertexIdx);
         Point2DReadOnly endVertex = polygon.getVertex(nextIdx);

         if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(pointOutside, pointInside, startVertex, endVertex, intersectionToPack))
            return startIndex;

         vertexIdx = nextIdx;
      }

      return -1;
   }

}
