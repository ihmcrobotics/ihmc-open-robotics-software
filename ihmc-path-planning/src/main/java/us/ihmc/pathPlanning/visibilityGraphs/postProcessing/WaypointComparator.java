package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.Comparator;

class WaypointComparator implements Comparator<Point2DReadOnly>
{
   private final Point2D startPoint = new Point2D();
   private final Point2D endPoint = new Point2D();

   public void setStartPoint(Point2DReadOnly startPoint)
   {
      this.startPoint.set(startPoint);
   }

   public void setEndPoint(Point2DReadOnly endPoint)
   {
      this.endPoint.set(endPoint);
   }

   @Override
   public int compare(Point2DReadOnly pointA, Point2DReadOnly pointB)
   {
      double distanceA = EuclidGeometryTools.percentageAlongLineSegment2D(pointA, startPoint, endPoint);
      double distanceB = EuclidGeometryTools.percentageAlongLineSegment2D(pointB, startPoint, endPoint);
      return Double.compare(distanceA, distanceB);
   }
}
