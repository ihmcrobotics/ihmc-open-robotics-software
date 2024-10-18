package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Plane3DBasics;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FixedFramePlane3DBasics extends FramePlane3DReadOnly, Plane3DBasics
{
   @Override
   FixedFramePoint3DBasics getPoint();

   @Override
   FixedFrameUnitVector3DBasics getNormal();

   /**
    * Redefines this plane with a new point and a new normal.
    *
    * @param referenceFrame the reference frame in which the given plane is expressed.
    * @param pointOnPlaneX the new x-coordinate of the point on this plane.
    * @param pointOnPlaneY the new y-coordinate of the point on this plane.
    * @param pointOnPlaneZ the new z-coordinate of the point on this plane.
    * @param planeNormalX  the new x-component of the normal of this plane.
    * @param planeNormalY  the new y-component of the normal of this plane.
    * @param planeNormalZ  the new z-component of the normal of this plane.
    */
   default void set(ReferenceFrame referenceFrame, double pointOnPlaneX, double pointOnPlaneY, double pointOnPlaneZ, double planeNormalX, double planeNormalY, double planeNormalZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pointOnPlaneX, pointOnPlaneY, pointOnPlaneZ, planeNormalX, planeNormalY, planeNormalZ);
   }

   /**
    * Sets this plane to be the same as the given plane.
    *
    * @param other the other plane to copy. Not modified.
    */
   default void set(FramePlane3DReadOnly other)
   {
      checkReferenceFrameMatch(other.getReferenceFrame());
      Plane3DBasics.super.set(other);
   }

   /**
    * Sets this plane to be the same as the given plane, and then transforms the data to match the current frame {@link #getReferenceFrame()}.
    *
    * @param other the other plane to copy. Not modified.
    */
   default void setMatchingFrame(FramePlane3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this plane to be the same as the given plane.
    *
    * @param referenceFrame the reference frame in which the given plane is expressed.
    * @param other the other plane to copy. Not modified.
    */
   default void set(ReferenceFrame referenceFrame, Plane3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Plane3DBasics.super.set(other);
   }

   /**
    * Sets this plane to be the same as the given plane, and then transforms the data to match the current frame {@link #getReferenceFrame()}.
    *
    * @param referenceFrame the reference frame in which the given plane is expressed.
    * @param other the other plane to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Plane3DReadOnly other)
   {
      Plane3DBasics.super.set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Redefines this plane such that it goes through the three given points.
    *
    * @param firstPointOnPlane  first point on this plane. Not modified.
    * @param secondPointOnPlane second point on this plane. Not modified.
    * @param thirdPointOnPlane  second point on this plane. Not modified.
    * @return {@code true} if the method succeeded, {@code false} if the method failed to estimate the
    *         normal.
    */
   default boolean set(FramePoint3DReadOnly firstPointOnPlane, FramePoint3DReadOnly secondPointOnPlane, FramePoint3DReadOnly thirdPointOnPlane)
   {
      checkReferenceFrameMatch(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
      return Plane3DBasics.super.set(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }

   /**
    * Redefines this plane with a new point and a new normal.
    *
    * @param pointOnPlane new point on this plane. Not modified.
    * @param planeNormal  new normal of this plane. Not modified.
    */
   default void set(FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      checkReferenceFrameMatch(pointOnPlane, planeNormal);
      Plane3DBasics.super.set(pointOnPlane, planeNormal);
   }
}
