package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * {@code FrameVector2D} is a 2D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector2D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frame.
 * </p>
 * <p>
 * Because a {@code FrameVector2D} extends {@code Vector2DBasics}, it is compatible with methods
 * only requiring {@code Vector2DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector2D}.
 * </p>
 */
public class FrameVector2D extends FrameTuple2D<FrameVector2D, Vector2D> implements FrameVector2DReadOnly, Vector2DBasics
{
   private static final long serialVersionUID = -610124454205790361L;

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector2D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Vector2D());
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, new Vector2D(x, y));
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y} in order from
    * the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      this(referenceFrame, new Vector2D(vectorArray));
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple2DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple2DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(referenceFrame, new Vector2D(tuple2DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector2D(FrameTuple2DReadOnly other)
   {
      this(other.getReferenceFrame(), new Vector2D(other));
   }

   /**
    * Creates a new frame vector and initializes it to x and y components of
    * {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector2D(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(frameTuple3DReadOnly.getReferenceFrame(), new Vector2D(frameTuple3DReadOnly));
   }

   public static FrameVector2D generateRandomFrameVector2d(Random random, ReferenceFrame zUpFrame)
   {
      double randomAngle = RandomNumbers.nextDouble(random, -Math.PI, Math.PI);

      FrameVector2D randomVector = new FrameVector2D(zUpFrame, Math.cos(randomAngle), Math.sin(randomAngle));

      return randomVector;
   }

   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public void setAndNormalize(FrameVector2DReadOnly other)
   {
      tuple.setAndNormalize(other);
   }

   /**
    * Calculates and returns the value of the cross product of this frame vector with {@code other}.
    *
    * @param other the second term in the cross product. Not modified.
    * @return the value of the cross product.
    */
   public double cross(FrameVector2DReadOnly other)
   {
      return FrameVector2DReadOnly.super.cross(other);
   }

   /**
    * Gets the read-only reference to the vector used in {@code this}.
    *
    * @return the vector of {@code this}.
    */
   public Vector2D getVector()
   {
      return tuple;
   }
}
