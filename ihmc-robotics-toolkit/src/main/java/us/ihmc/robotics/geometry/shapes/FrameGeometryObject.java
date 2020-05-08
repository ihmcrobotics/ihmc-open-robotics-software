package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * {@code FrameGeometryObject} is the base implementation of a general {@code GeometryObject}
 * associated with a {@code ReferenceFrame}.
 * <p>
 * Once initialized in a given reference frame, the geometry object can be transformed such that the
 * transformation represents the same geometry object viewed from another reference frame
 * perspective.
 * </p>
 *
 * @param <F> the final type of the extension of this abstract class.
 * @param <G> the final type of the geometry object to be used with this frame geometry object.
 */
public abstract class FrameGeometryObject<F extends FrameGeometryObject<F, G>, G extends GeometryObject<G>> implements ReferenceFrameHolder, GeometryObject<F>
{
   /** The frameless geometry object that is to be extended with a frame by using this class. */
   private final G geometryObject;
   /** The reference frame currently associated with the geometry object. */
   protected ReferenceFrame referenceFrame;

   /**
    * Creates a new frame geometry object.
    * <p>
    * The given {@code geometryObject}'s reference is saved internally for performing all the future
    * operations with this {@code FrameGeometryObject}.
    * </p>
    * <p>
    * The reference frame is initialized to {@link ReferenceFrame#getWorldFrame()}.
    * </p>
    *
    * @param geometryObject the geometry object that is to be used internally. Reference saved. Will be
    *                       modified.
    */
   public FrameGeometryObject(G geometryObject)
   {
      this(ReferenceFrame.getWorldFrame(), geometryObject);
   }

   /**
    * Creates a new frame geometry object and initializes its current reference frame.
    * <p>
    * The given {@code geometryObject}'s reference is saved internally for performing all the future
    * operations with this {@code FrameGeometryObject}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given geometry object is expressed
    *                       in.
    * @param geometryObject the geometry object that is to be used internally. Reference saved. Will be
    *                       modified.
    */
   public FrameGeometryObject(ReferenceFrame referenceFrame, G geometryObject)
   {
      this.geometryObject = geometryObject;
      this.referenceFrame = referenceFrame;
   }

   /**
    * Reset the values of the geometry object.
    * <p>
    * This method does affect the reference frame.
    * </p>
    */
   @Override
   public final void setToZero()
   {
      geometryObject.setToZero();
   }

   /**
    * Reset the values of the geometry object and changes the current reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with the geometry object.
    */
   public final void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToZero();
   }

   /**
    * Invalidate the geometry object by setting its values to {@link Double#NaN}.
    * <p>
    * This method does affect the reference frame.
    * </p>
    */
   @Override
   public final void setToNaN()
   {
      geometryObject.setToNaN();
   }

   /**
    * Invalidate the geometry object by setting its values to {@link Double#NaN} and changes the
    * current reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with the geometry object.
    */
   public final void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
   }

   /**
    * Tests if the geometry object contains at least one value equal to {@link Double#NaN}.
    *
    * @return {@code true} if the geometry object contains at least one value equal to
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public final boolean containsNaN()
   {
      return geometryObject.containsNaN();
   }

   /**
    * Copies the values from the given {@code geometryObject} into the geometry object in {@code this}.
    * <p>
    * Note that not reference frame check can be performed. If possible, prefer using other methods and
    * taking in either a {@code ReferenceFrame} or another {@code FrameGeometryObject}.
    * </p>
    *
    * @param geometryObject the geometry object used to update the geometry object in {@code this}. Not
    *                       modified.
    */
   public final void set(G geometryObject)
   {
      this.geometryObject.set(geometryObject);
   }

   /**
    * Copies the values from the given {@code geometryObject} into the geometry object in {@code this}.
    * <p>
    * The reference frame passed in, represents the coordinate system in which {@code geometryObject}
    * is expressed. This method ensures that the current reference frame in {@code this} is equal to
    * {@code referenceFrame}. If the two reference frames differ, this method throws a
    * {@link ReferenceFrameMismatchException}. If instead of throwing an exception this object's frame
    * should be changed, use {@link #setIncludingFrame(ReferenceFrame, GeometryObject)}.
    * </p>
    *
    * @param referenceFrame the coordinate system in which {@code geometryObject} is expressed.
    * @param geometryObject the geometry object used to update the geometry object in {@code this}. Not
    *                       modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   public final void set(ReferenceFrame referenceFrame, G geometryObject)
   {
      checkReferenceFrameMatch(referenceFrame);
      this.geometryObject.set(geometryObject);
   }

   /**
    * Copies the values from the given {@code geometryObject} into the geometry object in {@code this}
    * and changes this reference frame to {@code referenceFrame}.
    * <p>
    * The reference frame passed in, represents the coordinate system in which {@code geometryObject}
    * is expressed.
    * </p>
    *
    * @param referenceFrame the coordinate system in which {@code geometryObject} is expressed.
    * @param geometryObject the geometry object used to update the geometry object in {@code this}. Not
    *                       modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, G geometryObject)
   {
      this.referenceFrame = referenceFrame;
      this.geometryObject.set(geometryObject);
   }

   /**
    * Sets the geometry object in {@code this} to the geometry object in {@code other}.
    * <p>
    * Before updating the geometry object in {@code this}, this method ensures that the reference
    * frames currently associated with {@code this} and {@code other} match. If the two reference
    * frames differ, this method throws a {@link ReferenceFrameMismatchException}. If instead of
    * throwing an exception this object's frame should be changed, use
    * {@link #setIncludingFrame(FrameGeometryObject)}.
    * </p>
    *
    * @param other the other frame geometry object used to update the geometry object in {@code this}.
    *              Not modified.
    */
   @Override
   public final void set(F other)
   {
      checkReferenceFrameMatch(other);
      geometryObject.set(other.getGeometryObject());
   }

   /**
    * Sets both the reference frame and geometry object in {@code this} to {@code other}'s reference
    * frame and geometry object.
    *
    * @param other the other frame geometry object used to update {@code this}. Not modified.
    */
   public final void setIncludingFrame(F other)
   {
      referenceFrame = other.getReferenceFrame();
      geometryObject.set(other.getGeometryObject());
   }

   /**
    * Sets this frame geometry object to zero to the given reference frame, then changes back to this
    * frame geometry object's current frame.
    * <p>
    * Practically, this method can be used to store the transformation information from
    * {@code referenceFrame} to this current reference frame into this geometry object. How the
    * information is stored depends on the type of the geometry object. For instance, if the geometry
    * object is a {@code Point3D}, information retained from the transformation is the translation part
    * only.
    * </p>
    *
    * @param referenceFrame the reference frame of interest.
    */
   public final void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame thisReferenceFrame = getReferenceFrame();
      setToZero(referenceFrame);
      changeFrame(thisReferenceFrame);
   }

   /**
    * This is the main feature of a {@code FrameGeometryObject}:
    * <p>
    * Transforms this geometry object such that the transformation represents the same geometry but
    * from the perspective of another reference frame: {@code desiredFrame}. Once the geometry object
    * is transformed, the reference frame is updated to {@code desiredFrame}. In the case,
    * {@code this.referenceFrame == desiredFrame}, this method does nothing.
    * </p>
    *
    * @param desiredFrame the reference frame in which the geometry object is to be expressed.
    */
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      referenceFrame.transformFromThisToDesiredFrame(desiredFrame, this);
      referenceFrame = desiredFrame;
   }

   /**
    * Transforms the geometry object using the given {@code transform}.
    * <p>
    * This method does affect the reference frame.
    * </p>
    *
    * @param transform the transform to use on the geometry object in {@code this}. Not modified.
    */
   @Override
   public final void applyTransform(Transform transform)
   {
      geometryObject.applyTransform(transform);
   }

   /**
    * Transform the geometry object using the inverse of the given {@code transform}.
    * <p>
    * This method does affect the reference frame.
    * </p>
    *
    * @param transform the transform to use on the geometry object in {@code this}. Not modified.
    */
   @Override
   public final void applyInverseTransform(Transform transform)
   {
      geometryObject.applyInverseTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Packs the geometry object in {@code this} into the given {@code geometryToPack}.
    *
    * @param geometryObjectToPack the other geometry object used to stored the current values of the
    *                             geometry object in {@code this}. Modified.
    */
   public void get(G geometryObjectToPack)
   {
      geometryObjectToPack.set(this.geometryObject);
   }

   /**
    * Gets the reference to the geometry object held by {@code this}.
    *
    * @return the reference to the geometry object in {@code this}.
    */
   public final G getGeometryObject()
   {
      return geometryObject;
   }

   /**
    * Tests on a per component basis if {@code this} and {@code other} are equal to an {@code epsilon}.
    * <p>
    * First tests if {@code this} and {@code other} are currently expressed in the same reference
    * frame, if not, this returns {@code false}. Then, tests if {@code this.geometryObject} is equal to
    * {@code other.geometryObject} to an {@code epsilon}. The test is usually achieved on a per
    * component basis. Sometimes a failing test does not necessarily mean that the two objects are
    * different in a geometric way.
    * </p>
    *
    * @param other   the other object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   @Override
   public final boolean epsilonEquals(F other, double epsilon)
   {
      if (referenceFrame != other.referenceFrame)
         return false;
      return geometryObject.epsilonEquals(other.getGeometryObject(), epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same geometry to an {@code epsilon}.
    * <p>
    * This returns {@code false} if the two frame geometries are not expressed in the same reference
    * frame.
    * </p>
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will be
    * considered geometrically equal if they are at a distance from each other that is less or equal
    * than {@code epsilon}.
    * </p>
    * <p>
    * Two frame geometries that are geometrically equal are not necessarily epsilon equals and vice
    * versa.
    * </p>
    *
    * @param other   the other geometry object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing the two objects, usually refers to a distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(F other, double epsilon)
   {
      if (referenceFrame != other.referenceFrame)
         return false;
      return geometryObject.geometricallyEquals(other.getGeometryObject(), epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameGeometryObject)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @SuppressWarnings("unchecked")
   @Override
   public final boolean equals(Object object)
   {
      try
      {
         return equals((F) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * First, tests if {@code this} and {@code other} are currently expressed in the same reference
    * frame, if not, this method returns {@code false}. Then, tests if {@code this.geometryObject} is
    * exactly equal to {@code other.geometryObject}. The test is usually achieved on a per component
    * basis. Sometimes a failing test does not necessarily mean that the two objects are different in a
    * geometric way.
    *
    * @param other the other frame geometry object to compare against this. Not modified.
    * @return {@code true} if the two objects are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public final boolean equals(F other)
   {
      try
      {
         if (referenceFrame != other.referenceFrame)
            return false;
         return geometryObject.equals(other.getGeometryObject());
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame geometry object.
    * <p>
    * The returned {@code String} is the concatenation of the geometry object and reference frame
    * {@code String} representations.
    * </p>
    *
    * @return the {@code String} representation of this frame geometry object.
    */
   @Override
   public String toString()
   {
      return geometryObject + "-" + referenceFrame;
   }
}