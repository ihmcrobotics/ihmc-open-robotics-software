package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;

import java.util.*;

public class ConcaveHullCutModel
{
   private CircularArrayList<ConcaveHullCutPoint> points = new CircularArrayList<>();
   private final CircularArrayList<ConcaveHullCutPoint> intersectionsFromLeftToRight;
   private final LinkedHashMap<Integer, ConcaveHullCutPoint> nextVertexToIntersectionMap;

   public ConcaveHullCutModel(List<Point2D> concaveHullVertices,
                              Line2DReadOnly cuttingLine,
                              Vector2DReadOnly upDirection,
                              Map<Integer, Point2D> intersections)
   {
      Pose2dReferenceFrame cuttingLineFrame = new Pose2dReferenceFrame("cuttingLineFrame", ReferenceFrame.getWorldFrame());
      double angle = EuclidGeometryTools.angleFromFirstToSecondVector2D(0.0, 1.0, upDirection.getX(), upDirection.getY()); // angle from yUp
      cuttingLineFrame.setPoseAndUpdate(cuttingLine.getPoint(), angle);

      intersectionsFromLeftToRight = new CircularArrayList<>();
      for (Map.Entry<Integer, Point2D> intersection : intersections.entrySet())
      {
         FramePoint2D frameIntersection = new FramePoint2D(ReferenceFrame.getWorldFrame(), intersection.getValue());
         frameIntersection.changeFrame(cuttingLineFrame);
         intersectionsFromLeftToRight.add(new ConcaveHullCutPoint(frameIntersection, intersection.getKey()));
      }
      intersectionsFromLeftToRight.sort(Comparator.comparingDouble(ConcaveHullCutPoint::getX));
      for (ConcaveHullCutPoint intersection : intersectionsFromLeftToRight)
      {
         intersection.getAsFramePoint().changeFrame(ReferenceFrame.getWorldFrame()); // change back to world frame
      }

      nextVertexToIntersectionMap = new LinkedHashMap<>();
      for (ConcaveHullCutPoint concaveHullCutPoint : intersectionsFromLeftToRight)
      {
         nextVertexToIntersectionMap.put(concaveHullCutPoint.getNextVertex(), concaveHullCutPoint);
      }

      // build vertex list with intersections included
      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         points.add(new ConcaveHullCutPoint(concaveHullVertices.get(i)));

         int nextVertex = EuclidGeometryPolygonTools.next(i, concaveHullVertices.size());
         if (nextVertexToIntersectionMap.containsKey(nextVertex))
         {
            points.add(nextVertexToIntersectionMap.get(nextVertex));
            --i;
         }
      }

      // reorder vertex-intersection list to start at leftmost intersection
      int indexOfLeftmostIntersection = 0;
      for (int i = 0; i < points.size(); i++)
      {
         if (points.get(i).isIntersection() && points.get(i).getAsFramePoint().equals(intersectionsFromLeftToRight.get(0)))
         {
            indexOfLeftmostIntersection = i;
         }
      }
      points.reindexTo(indexOfLeftmostIntersection);
   }

   public int indexOfIntersectionToLeft(int indexKnownToBeAnIntersection)
   {
      for (int i = 0; i < points.size(); i++)
      {
         if (i == indexKnownToBeAnIntersection)
         {
            int intersectionIndex = intersectionsFromLeftToRight.indexOf(points.get(i));
            return intersectionsFromLeftToRight.getPreviousIndex(intersectionIndex);
         }
      }

      throw new RuntimeException("Shouldn't get here");
   }

   public boolean allIntersectionsVisited()
   {
      for (ConcaveHullCutPoint point : intersectionsFromLeftToRight)
      {
         if (!point.getVisited())
         {
            return false;
         }
      }
      return true;
   }

   public int indexOfFirstUnvisitedIntersection()
   {
      for (int i = 0; i < intersectionsFromLeftToRight.size(); i++)
      {
         ConcaveHullCutPoint concaveHullCutPoint = intersectionsFromLeftToRight.get(i);

         if (!concaveHullCutPoint.getVisited())
         {
            return i;
         }
      }

      throw new RuntimeException("Somehow the unvisited vertex didn't equal any in the combined list. Bug!");
   }

   public CircularArrayList<ConcaveHullCutPoint> getPoints()
   {
      return points;
   }

   public CircularArrayList<ConcaveHullCutPoint> getIntersectionsFromLeftToRight()
   {
      return intersectionsFromLeftToRight;
   }

   public LinkedHashMap<Integer, ConcaveHullCutPoint> getNextVertexToIntersectionMap()
   {
      return nextVertexToIntersectionMap;
   }
}
