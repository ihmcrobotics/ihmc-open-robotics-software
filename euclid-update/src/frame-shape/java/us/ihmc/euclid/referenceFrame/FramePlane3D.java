package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DBasics;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@link FramePlane3D} is a 3D plane expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Plane3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@link FramePlane3D}. This allows, for instance, to enforce, at runtime, that operations on planes
 * occur in the same coordinate system. Also, via the method {@link #changeFrame(ReferenceFrame)},
 * one can easily calculates the value of a point in different reference frames.
 * </p>
 * <p>
 * Because a {@link FramePlane3D} extends {@link Plane3DBasics}, it is compatible with methods only
 * requiring {@link Plane3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@link FramePlane3D}.
 * </p>
 */
public class FramePlane3D implements FramePlane3DBasics, Settable<FramePlane3D>
{
   /** The reference frame in which this plane is expressed. */
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics point = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFrameUnitVector3DBasics normal = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(this, Axis3D.Z);

   /**
    * Default constructor that initializes its {@code point} to zero, its {@code normal} to
    * {@link Axis3D#Z} and the reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FramePlane3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new plane and initializes both {@link #point} and {@link #normal} to zero and the
    * reference frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this plane.
    */
   public FramePlane3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new plane and initializes both {@link #point} and {@link #normal} to the {@param plane3D} and the
    * reference frame to the given {@param referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this plane.
    * @param plane3D the initial value for this plane.
    */
   public FramePlane3D(ReferenceFrame referenceFrame, Plane3DReadOnly plane3D)
   {
      setIncludingFrame(referenceFrame, plane3D);
   }

   /**
    * Creates a new plane and initializes both {@link #point} and {@link #normal} to the {@param plane3D} and the
    * reference frame to the given {@param plane3D}.
    *
    * @param framePlane3D the initial value and frame for this plane.
    */
   public FramePlane3D(FramePlane3DReadOnly framePlane3D)
   {
      setIncludingFrame(framePlane3D);
   }

   /**
    * Creates a new plane and initializes both {@link #point} and {@link #normal} to the {@param point} and {@param normal} frame and value.
    *
    * @param point the initial point for this plane.
    * @param normal the initial normal for this plane.
    * @throws us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException if the reference frames of {@param point} and {@param normal} are not the same.
    */
   public FramePlane3D(FramePoint3DReadOnly point, FrameVector3DReadOnly normal)
   {
      setIncludingFrame(point, normal);
   }

   /**
    * Creates a new plane and initializes both {@link #point} and {@link #normal} to the {@param point} and {@param normal} frame and value.
    *
    * @param referenceFrame the initial
    * @param point the initial point for this plane.
    * @param normal the initial normal for this plane.
    */
   public FramePlane3D(ReferenceFrame referenceFrame, Point3DReadOnly point, Vector3DReadOnly normal)
   {
      setIncludingFrame(referenceFrame, point, normal);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FramePlane3D other)
   {
      FramePlane3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPoint()
   {
      return point;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameUnitVector3DBasics getNormal()
   {
      return normal;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePlane3DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame plane 3D as follows:<br>
    * Plane 3D: point = (x, y, z), normal = (x, y, z)-worldFrame
    *
    * @return the {@code String} representing this plane 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
   /**
    * Calculates and returns a hash code value from the value of each component of this plane 3D.
    *
    * @return the hash code value for this plane 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, normal);
   }
}
