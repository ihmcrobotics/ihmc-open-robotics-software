package us.ihmc.robotics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.List;

public class EuclidGeometryPolygonMissingTools
{
   public static boolean doLineSegment2DAndConvexPolygon2DIntersect(Point2DReadOnly lineSegmentStart,
                                                                        Point2DReadOnly lineSegmentEnd,
                                                                        List<? extends Point2DReadOnly> convexPolygon2D,
                                                                        int numberOfVertices)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return false;

      if (numberOfVertices == 1)
         return EuclidGeometryTools.isPoint2DOnLineSegment2D(convexPolygon2D.get(0), lineSegmentStart, lineSegmentEnd);

      if (numberOfVertices == 2)
         return EuclidGeometryTools.doLineSegment2DsIntersect(convexPolygon2D.get(0), convexPolygon2D.get(1), lineSegmentStart, lineSegmentEnd);


      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(EuclidGeometryPolygonTools.next(edgeIndex, numberOfVertices));

         if (EuclidGeometryTools.doLineSegment2DsIntersect(edgeStart, edgeEnd, lineSegmentStart, lineSegmentEnd))
            return true;

      }

      return false;
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }
}
