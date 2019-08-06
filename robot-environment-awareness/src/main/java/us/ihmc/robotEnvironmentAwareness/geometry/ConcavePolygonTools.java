package us.ihmc.robotEnvironmentAwareness.geometry;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.ConvexPolygonCropResult;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

import java.util.ArrayList;
import java.util.List;

public class ConcavePolygonTools
{

   public static List<ConcaveHull> cropPolygonToAboveLine(ConcaveHull concaveHullToCrop,
                                                          Line2DReadOnly cuttingLine,
                                                          Vector2DReadOnly upDirection)
   {
      ArrayList<ConcaveHull> resultingConcaveHulls = new ArrayList<>();
      if (concaveHullToCrop.getNumberOfVertices() < 5)
      {
         // must be convex, revert to convex polygon crop
         ConvexPolygon2D convexPolygonToCrop = new ConvexPolygon2D();
         for (Point2D concaveHullVertex : concaveHullToCrop.getConcaveHullVertices())
         {
            convexPolygonToCrop.addVertex(concaveHullVertex);
         }
         convexPolygonToCrop.update();
         ConvexPolygon2D croppedPolygonToPack = new ConvexPolygon2D();
         ConvexPolygonCropResult result = ConvexPolygonTools.cropPolygonToAboveLine(convexPolygonToCrop,
                                                                                    cuttingLine,
                                                                                    upDirection,
                                                                                    croppedPolygonToPack);
         if (result != ConvexPolygonCropResult.REMOVE_ALL)
         {
            ConcaveHull concaveHullToReturn = new ConcaveHull(croppedPolygonToPack.getVertexBufferView());
            resultingConcaveHulls.add(concaveHullToReturn);
         }
         return resultingConcaveHulls;
      }

      // assert vertices 5 or greater
      if (concaveHullToCrop.getNumberOfVertices() < 5)
         throw new RuntimeException("This polygon must be convex and shouldn't have gotten this far.");

      // find intersections (number of intersections can be as high as n-1)
      ArrayList<Pair<Integer, Point2D>> intersections = new ArrayList<>(); // index after intersection; intersection point
      List<Point2D> concaveHullVertices = concaveHullToCrop.getConcaveHullVertices();
      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         int nextVertex = EuclidGeometryPolygonTools.next(i, concaveHullVertices.size());

         /**
          * Do a super low level intersection to make obvious all potential cases. This may not be necessary,
          * but until that's for sure it helps to see these potential corner cases laid out.
          */
         double lineSegmentDirectionX = concaveHullVertices.get(nextVertex).getX() - concaveHullVertices.get(i).getX();
         double lineSegmentDirectionY = concaveHullVertices.get(nextVertex).getY() - concaveHullVertices.get(i).getY();
         double percentage = EuclidCoreMissingTools.percentageOfIntersectionBetweenTwoLine2DsInfCase(concaveHullVertices.get(i).getX(),
                                                                                                     concaveHullVertices.get(i).getY(),
                                                                                                     lineSegmentDirectionX,
                                                                                                     lineSegmentDirectionY,
                                                                                                     cuttingLine.getPoint().getX(),
                                                                                                     cuttingLine.getPoint().getY(),
                                                                                                     cuttingLine.getDirection().getX(),
                                                                                                     cuttingLine.getDirection().getY());

         if (percentage == Double.NaN) // non-intersecting parallel
         {
            // do nothing
         }
         else if (percentage == Double.POSITIVE_INFINITY) // colinear edge intersection, store both vertices as intersections
         {

         }
         else
         {
            Point2D intersection = new Point2D();
            intersections.add(Pair.of(nextVertex, intersection));
         }
      }

      // filter out colinear intersections?

      boolean vertex0IsAbove = EuclidGeometryTools.isPoint2DInFrontOfRay2D(concaveHullToCrop.getVertex(0), cuttingLine.getPoint(), upDirection);
      LogTools.debug("Intersection count: {} vertex(0) is above line: {}", intersections.size(), vertex0IsAbove);

      if (intersections.size() == 0) // first edge case: no intersections: keep all or remove all
      {
         if (vertex0IsAbove)
         {
            resultingConcaveHulls.add(new ConcaveHull(concaveHullToCrop));
            return resultingConcaveHulls;
         }
         else
         {
            return resultingConcaveHulls;
         }
      }
      else if (intersections.size() == 1)
      {
         // firstIntersectionToPack is packed with only intersection
         if (concaveHullToCrop.getNumberOfVertices() > 1)
         {
            // isPoint2DInFrontOfRay2D returns true for on as well. Check any two vertices. One is on the line.
            boolean isOnOrAboveTwo = EuclidGeometryTools.isPoint2DInFrontOfRay2D(concaveHullToCrop.getVertex(1), cuttingLine.getPoint(), upDirection);

            if (vertex0IsAbove && isOnOrAboveTwo)
            {
               resultingConcaveHulls.add(new ConcaveHull(concaveHullToCrop));
               return resultingConcaveHulls;
            }
            else
            {
               return resultingConcaveHulls;
            }
         }
         else
         {
            return resultingConcaveHulls;
         }
      }
      else
      {
         // must handle and colinear intersections


      }

      // number of returned hulls may be as high as (n-1)/2
      return resultingConcaveHulls;
   }
}
