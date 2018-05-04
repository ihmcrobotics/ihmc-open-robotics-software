package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

/**
 * A Point2d in ConvexPolygon coordinate is defined by  (eccentricity, angle) similar to the polar coordinate system
 * However, eccentricity defines the relative distance to origin w.r.t. maximum possible distance on the same epipolar line within the polygon.
 * Essentially, eccentricity = 1 means the point is on the polygon boundary.
 * @author tingfan
 *
 */
public class Point2dInConvexPolygon2d extends Point2D
{
   private static final long serialVersionUID = 5818978949209007789L;
   protected  ConvexPolygon2D polygon;
   private final Point2D origin = new Point2D(0,0);


   public Point2dInConvexPolygon2d(ConvexPolygon2D polygon, double x, double y)
   {
      super(x,y);
      this.polygon = polygon;
   }

   public boolean isPointInsidePolygon()
   {
      return polygon.isPointInside(this);
   }


   public void setEccentricity(double r)
   {
      r=Math.max(r, 1e-5);
      double e = getEccentricity();
      scale(r/e);
   }

   public void setAngle(double angle)
   {
      Point2DBasics point = findEdgePoint(angle);
      point.scale(getEccentricity());
      set(point);
   }


      public double getAngle()
   {
      return Math.atan2(getY(),getX());
   }

   public double getEccentricity()
   {
      Point2DBasics edgePoint = findEdgePoint(getX(), getY());
      return Math.max(1e-3, distance(origin) / edgePoint.distance(origin));
   }

   private Point2DBasics findEdgePoint(double angle)
   {
      return findEdgePoint(Math.cos(angle), Math.sin(angle));
   }

   private Point2DBasics findEdgePoint(double x, double y)
   {
      if (x==0 && y==0)
         x=1; //as eccentricity=0
      Line2D ray = new Line2D(new Point2D(0,0), new Vector2D(x,y));
      Point2DBasics[] edgePoints = polygon.intersectionWithRay(ray);
      if(edgePoints.length!=1)
         throw new RuntimeException("intersecting points should be 1, but we get" + edgePoints.length);
      return edgePoints[0];
   }

}
