package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FramePlane3DBasics extends FixedFramePlane3DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this plane 3D without updating or modifying its point or direction.
    *
    * @param referenceFrame the new reference frame for this frame plane 3D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the point of this plane to zero, its normal to {@link Axis3D#Z}, and sets the current
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this plane 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the point and direction parts of this plane 3D to {@link Double#NaN} and sets the current
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this plane 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Redefines this plane with a new point, a new normal vector, and a new reference frame.
    *
    * @param referenceFrame the new reference frame for this frame plane.
    * @param pointOnPlaneX   the new x-coordinate of the point on this plane.
    * @param pointOnPlaneY   the new y-coordinate of the point on this plane.
    * @param pointOnPlaneZ   the new z-coordinate of the point on this plane.
    * @param planeNormalX   the new x-component of the normal of this plane.
    * @param planeNormalY   the new y-component of the normal of this plane.
    * @param planeNormalZ   the new z-component of the normal of this plane.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame,
                                  double pointOnPlaneX,
                                  double pointOnPlaneY,
                                  double pointOnPlaneZ,
                                  double planeNormalX,
                                  double planeNormalY,
                                  double planeNormalZ)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ, planeNormalX, planeNormalY, planeNormalZ);
   }

   /**
    * Sets this plane to be the same as the given plane.
    *
    * @param referenceFrame  the new reference frame for this frame plane.
    * @param plane3DReadOnly the plane to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Plane3DReadOnly plane3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(plane3DReadOnly);
   }

   /**
    * Redefines this plane with a new point, a new normal vector, and a new reference frame.
    *
    * @param referenceFrame the new reference frame for this frame plane.
    * @param pointOnPlane   new point on this plane. Not modified.
    * @param planeNormal    new direction of this plane. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      setReferenceFrame(referenceFrame);
      set(pointOnPlane, planeNormal);
   }

   /**
    * Redefines this plane such that it intersects the three given points in the given reference frame.
    *
    * @param referenceFrame     the new reference frame for this frame plane.
    * @param firstPointOnPlane  first point on this plane. Not modified.
    * @param secondPointOnPlane second point on this plane. Not modified.
    * @param thirdPointOnPlane  second point on this plane. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame,
                                  Point3DReadOnly firstPointOnPlane,
                                  Point3DReadOnly secondPointOnPlane,
                                  Point3DReadOnly thirdPointOnPlane)
   {
      setReferenceFrame(referenceFrame);
      set(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }

   /**
    * Sets this plane to be the same as the given plane.
    *
    * @param other the other plane to copy. Not modified.
    */
   default void setIncludingFrame(FramePlane3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Redefines this plane with a new point, a new direction vector, and a new reference frame.
    *
    * @param pointOnPlane  new point on this plane. Not modified.
    * @param normalVector new direction of this plane. Not modified.
    * @throws ReferenceFrameMismatchException if {@code pointOnPlane} and {@code normalVector} are not
    *       expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly normalVector)
   {
      pointOnPlane.checkReferenceFrameMatch(normalVector);
      setIncludingFrame(pointOnPlane.getReferenceFrame(), pointOnPlane, normalVector);
   }

   /**
    * Redefines this plane such that it intersects the three given points in the given reference frame.
    *
    * @param firstPointOnPlane  first point on this plane. Not modified.
    * @param secondPointOnPlane second point on this plane. Not modified.
    * @param thirdPointOnPlane  second point on this plane. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPointOnPlane}, {@code secondPointOnPlane}, and {@code thirdPointOnPlane}
    *       are not expressed in the same reference frame
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstPointOnPlane, FramePoint3DReadOnly secondPointOnPlane, FramePoint3DReadOnly thirdPointOnPlane)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane, thirdPointOnPlane);
      setIncludingFrame(firstPointOnPlane.getReferenceFrame(), firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }
}
