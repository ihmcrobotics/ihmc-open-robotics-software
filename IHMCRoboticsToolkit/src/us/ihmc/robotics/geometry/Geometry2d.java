package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author Twan Koolen
 * @version 1.0
 */
public interface Geometry2d<T extends Geometry2d<T>> extends GeometryObject<T>
{
   // Orthogonal projection:
   public void orthogonalProjection(Point2DBasics tuple);

   public Point2D orthogonalProjectionCopy(Point2DReadOnly point);

   // Intersection:
   public Object intersectionWith(Line2d line);

   public Object intersectionWith(LineSegment2d lineSegment);

   public Object intersectionWith(ConvexPolygon2d convexPolygon);

   // Distance:
   public double distance(Point2DReadOnly point2d);

   public double distance(Line2d line);

   public double distance(LineSegment2d lineSegment);

   public double distance(ConvexPolygon2d convexPolygon);

   // Extra Transformations for being 2d:

   public void applyTransformAndProjectToXYPlane(Transform transform);

   public Geometry2d applyTransformCopy(Transform transform);

   public Geometry2d applyTransformAndProjectToXYPlaneCopy(Transform transform);

   void applyTransform(Transform transform);
}
