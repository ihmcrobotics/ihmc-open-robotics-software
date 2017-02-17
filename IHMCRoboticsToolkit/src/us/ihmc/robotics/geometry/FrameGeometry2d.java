package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public abstract class FrameGeometry2d<S extends FrameGeometry2d<S, T>, T extends GeometryObject<T>> extends AbstractFrameObject<S, T>
{
   public FrameGeometry2d(ReferenceFrame referenceFrame, T transformableDataObject)
   {
      super(referenceFrame, transformableDataObject);
   }

   // Orthogonal projection:
   public abstract void orthogonalProjection(FramePoint2d point);

   public abstract FramePoint2d orthogonalProjectionCopy(FramePoint2d point);

   // Intersection:
   public abstract Object intersectionWith(FrameLine2d line);

   public abstract Object intersectionWith(FrameLineSegment2d lineSegment);

   public abstract Object intersectionWith(FrameConvexPolygon2d convexPolygon);

   // Distance:
   public abstract double distance(FramePoint2d point);

   public abstract double distance(FrameLine2d line);

   public abstract double distance(FrameLineSegment2d lineSegment);

   public abstract double distance(FrameConvexPolygon2d convexPolygon);

   // Transformations:
   public abstract void applyTransformAndProjectToXYPlane(Transform transform);

   public abstract FrameGeometry2d<S, T> applyTransformCopy(Transform transform);

   public abstract FrameGeometry2d<S, T> applyTransformAndProjectToXYPlaneCopy(Transform transform);

   public abstract void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);

   public abstract FrameGeometry2d<S, T> changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame);
}
