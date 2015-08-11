package us.ihmc.robotics.geometry;

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
public abstract class FrameGeometry2d extends ReferenceFrameHolder
{
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
   public abstract void applyTransform(RigidBodyTransform transform);

   public abstract void applyTransformAndProjectToXYPlane(RigidBodyTransform transform);

   public abstract FrameGeometry2d applyTransformCopy(RigidBodyTransform transform);

   public abstract FrameGeometry2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform);

   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public abstract void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);

   public abstract FrameGeometry2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame);
}
