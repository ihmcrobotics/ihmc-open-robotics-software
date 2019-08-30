package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.ConvexPolygonCropResult;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

import java.util.*;

public class ConcavePolygonTools
{
   public static final int MIN_VERTICES_TO_HAVE_CONCAVITY = 4;

   public static List<ConcaveHull> cutPolygonToLeftOfLine(ConcaveHull concaveHullToCrop,
                                                          Line2DReadOnly cuttingLine)
   {
      Vector2D upDirection = new Vector2D(cuttingLine.getDirection());
      upDirection.normalize();
      upDirection.set(-upDirection.getY(), upDirection.getX());

      // filter hull vertices off of line
      ConcaveHull filteredConcaveHullToCrop = new ConcaveHull();
      for (Point2D point2D : concaveHullToCrop)
      {
         Point2D filteredVertex = new Point2D(point2D);
         double distance = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(point2D, cuttingLine.getPoint(), cuttingLine.getDirection());
         if (Math.abs(distance) < 1e-5) // move a little
         {
            Vector2D moveBy = new Vector2D();
            if (distance >= 0.0) // move just above
            {
               moveBy.set(upDirection);
            }
            else // if (distance < 0.0) // move just below
            {
               moveBy.set(-upDirection.getX(), -upDirection.getY());
            }
            moveBy.scale(1e-5);
            filteredVertex.add(moveBy);
         }
         filteredConcaveHullToCrop.addVertex(filteredVertex);
      }

      ArrayList<ConcaveHull> resultingConcaveHulls = new ArrayList<>();
      if (filteredConcaveHullToCrop.getNumberOfVertices() < MIN_VERTICES_TO_HAVE_CONCAVITY)
      {
         // must be convex, revert to convex polygon crop
         ConvexPolygon2D convexPolygonToCrop = new ConvexPolygon2D();
         for (Point2D concaveHullVertex : filteredConcaveHullToCrop.getConcaveHullVertices())
         {
            convexPolygonToCrop.addVertex(concaveHullVertex);
         }
         convexPolygonToCrop.update();
         ConvexPolygon2D croppedPolygonToPack = new ConvexPolygon2D();
         ConvexPolygonCropResult result = ConvexPolygonTools.cutPolygonToLeftOfLine(convexPolygonToCrop,
                                                                                    cuttingLine,
                                                                                    croppedPolygonToPack);
         if (result != ConvexPolygonCropResult.REMOVE_ALL)
         {
            ConcaveHull concaveHullToReturn = new ConcaveHull(croppedPolygonToPack.getVertexBufferView());
            resultingConcaveHulls.add(concaveHullToReturn);
         }
         return resultingConcaveHulls;
      }

      // assert vertices 4 or greater
      if (filteredConcaveHullToCrop.getNumberOfVertices() < MIN_VERTICES_TO_HAVE_CONCAVITY)
         throw new RuntimeException("This polygon must be convex and shouldn't have gotten this far.");

      // find intersections (number of intersections can be as high as n-1)
      Map<Integer, Point2D> intersections = new HashMap<>(); // index after intersection; intersection point
      List<Point2D> concaveHullVertices = filteredConcaveHullToCrop.getConcaveHullVertices();
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
         else // normal intersection
         {
            // check if contained by line segment

            if (percentage > 0.0 && percentage < 1.0)
            {
               Point2D intersection = new Point2D();
               intersection.interpolate(concaveHullVertices.get(i), concaveHullVertices.get(nextVertex), percentage);
               // TODO maybe discritize?
               intersection.setX(MathTools.roundToPrecision(intersection.getX(), 1e-6));
               intersection.setY(MathTools.roundToPrecision(intersection.getY(), 1e-6));
               intersections.put(nextVertex, intersection);
            }
         }
      }

      // filter out colinear edge intersections? did we just do this above?

      boolean vertex0IsAbove = EuclidGeometryTools.isPoint2DInFrontOfRay2D(filteredConcaveHullToCrop.getVertex(0), cuttingLine.getPoint(), upDirection);
      LogTools.debug("Intersection count: {} vertex(0) is above line: {}", intersections.size(), vertex0IsAbove);

      if (intersections.size() == 0) // first edge case: no intersections: keep all or remove all
      {
         if (vertex0IsAbove)
         {
            resultingConcaveHulls.add(new ConcaveHull(filteredConcaveHullToCrop));
            return resultingConcaveHulls;
         }
         else
         {
            return resultingConcaveHulls;
         }
      }
      else if (intersections.size() == 1) // second edge case: point intersection: keep all or remove all
      {
         // firstIntersectionToPack is packed with only intersection
         if (filteredConcaveHullToCrop.getNumberOfVertices() > 1)
         {
            // isPoint2DInFrontOfRay2D returns true for on as well. Check any two vertices. One is on the line.
            boolean isOnOrAboveTwo = EuclidGeometryTools.isPoint2DInFrontOfRay2D(filteredConcaveHullToCrop.getVertex(1), cuttingLine.getPoint(), upDirection);

            if (vertex0IsAbove && isOnOrAboveTwo)
            {
               resultingConcaveHulls.add(new ConcaveHull(filteredConcaveHullToCrop));
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
         // TODO handle colinear intersections

         ConcaveHullCutModel model = new ConcaveHullCutModel(concaveHullVertices, cuttingLine, upDirection, intersections);

         // while all intersections not visited
         while (!model.allIntersectionsVisited())
         {
            // find intersection to start at; find first unvisited intersection
            int firstUnvisitedVertex = model.indexOfFirstUnvisitedIntersection();

            ConcaveHull currentResultingConcaveHull = new ConcaveHull();
            // always add the first point
            currentResultingConcaveHull.addVertex(new Point2D(model.getPoints().get(firstUnvisitedVertex).getPoint()));
            model.getPoints().get(firstUnvisitedVertex).setVisited(true);

            int travellingIndex = firstUnvisitedVertex;

            final int ALONG_HULL = 5;
            final int TRAVERSE_CUT_LINE = 6;

            int drawState = ALONG_HULL;

            // while it's not back to the start
            do
            {
               if (drawState == ALONG_HULL) // if we are along hull, is next point intersection?
               {
                  travellingIndex = model.getPoints().getNextIndex(travellingIndex);

                  boolean nextPointIsIntersection = model.getPoints().get(travellingIndex).isIntersection();

                  if (nextPointIsIntersection)
                  {
                     drawState = TRAVERSE_CUT_LINE;
                  }
               }
               else // if (drawState == TRAVERSE_CUT_LINE)
               {
                  travellingIndex = model.indexOfIntersectionToLeft(travellingIndex);

                  drawState = ALONG_HULL;
               }

               if (travellingIndex != firstUnvisitedVertex)
               {
                  currentResultingConcaveHull.addVertex(new Point2D(model.getPoints().get(travellingIndex).getPoint()));
                  model.getPoints().get(travellingIndex).setVisited(true);
               }
            }
            while (travellingIndex != firstUnvisitedVertex);

            // finish up current resulting concave hull
            resultingConcaveHulls.add(currentResultingConcaveHull);
         }

         // number of returned hulls may be as high as (n-1)/2
         return resultingConcaveHulls;
      }
   }
}
