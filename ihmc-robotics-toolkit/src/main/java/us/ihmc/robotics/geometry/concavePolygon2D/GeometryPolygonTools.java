package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.List;

import static us.ihmc.commons.lists.ListWrappingIndexTools.next;
import static us.ihmc.commons.lists.ListWrappingIndexTools.previous;

public class GeometryPolygonTools
{
   public static boolean isClockwiseOrdered(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      double sumOfAngles = 0.0;

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.size(); vertexIndex++)
      {
         int previousVertexIndex = previous(vertexIndex, concaveHullVertices);
         int nextVertexIndex = next(vertexIndex, concaveHullVertices);

         Point2DReadOnly previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
         Point2DReadOnly nextVertex = concaveHullVertices.get(nextVertexIndex);

         double firstVectorX = vertex.getX() - previousVertex.getX();
         double firstVectorY = vertex.getY() - previousVertex.getY();
         double secondVectorX = nextVertex.getX() - vertex.getX();
         double secondVectorY = nextVertex.getY() - vertex.getY();
         sumOfAngles += angle(firstVectorX, firstVectorY, secondVectorX, secondVectorY);
      }

      return sumOfAngles <= 0.0;
   }

   public static boolean isSimplePolygon(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      // TODO implement some other versions of this algorithm that are faster (see https://www.webcitation.org/6ahkPQIsN)
      return isSimplePolygonBruteForce(concaveHullVertices);
   }

   public static boolean isSimplePolygonBruteForce(List<? extends Point2DReadOnly> concaveHullVertices)
   {
      int size = concaveHullVertices.size();
      for (int index = 0; index < size; index++)
      {
         int previousIndex = EuclidGeometryPolygonTools.previous(index, size);

         Point2DReadOnly segmentStart = concaveHullVertices.get(previousIndex);
         Point2DReadOnly segmentEnd = concaveHullVertices.get(index);

         int nextSegmentStart = EuclidGeometryPolygonTools.next(index, size);
         int nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, size);
         while (nextSegmentEnd != previousIndex)
         {
            if (EuclidGeometryTools.doLineSegment2DsIntersect(segmentStart,
                                                              segmentEnd,
                                                              concaveHullVertices.get(nextSegmentStart),
                                                              concaveHullVertices.get(nextSegmentEnd)))
            {
               return false;
            }

            nextSegmentStart = nextSegmentEnd;
            nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, size);
         }
      }

      return true;
   }

   private static double angle(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector)
   {
      return angle(firstVector.getX(), firstVector.getY(), secondVector.getX(), secondVector.getY());
   }

   private static double angle(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY)
   {
      // The sign of the angle comes from the cross product
      double crossProduct = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      // the magnitude of the angle comes from the dot product
      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY;

      return EuclidCoreTools.atan2(crossProduct, dotProduct);
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }
}
