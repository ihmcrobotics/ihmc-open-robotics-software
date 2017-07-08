package us.ihmc.robotics.geometry;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FrameTuple3D} is the base implementation for {@link FramePoint3D} and {@link FrameVector3D}.
 * <p>
 * In addition to representing a {@link Tuple3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameTuple3D}. This allows, for instance, to enforce, at runtime, that operations on
 * tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple3D} extends {@code Tuple3DBasics}, it is compatible with methods only
 * requiring {@code Tuple3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameTuple3D}.
 * </p>
 */
public abstract class FrameTuple3D<S extends FrameTuple3D<S, T>, T extends Tuple3DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple3DReadOnly, Tuple3DBasics, Serializable
{
   private static final long serialVersionUID = 7909025372897997396L;

   /** Tuple used to perform the operations. */
   protected final T tuple;

   /**
    * Creates a new frame tuple and initializes its current reference frame and tuple.
    * <p>
    * The given {@code geometryObject}'s reference is saved internally for performing all the future
    * operations with this {@code FrameTuple3D}.
    * </p>
    * 
    * @param referenceFrame the initial reference frame in which the given tuple is expressed in.
    * @param tuple the tuple that is to be used internally. Reference saved. Will be modified.
    */
   public FrameTuple3D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public final void setX(double x)
   {
      tuple.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public final void setY(double y)
   {
      tuple.setY(y);
   }

   /** {@inheritDoc} */
   @Override
   public final void setZ(double z)
   {
      tuple.setZ(z);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code frameTuple2DReadOnly}.
    * <p>
    * The z component remains unchanged.
    * </p>
    * 
    * @param frameTuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public void set(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple2DReadOnly);
      tuple.set(frameTuple2DReadOnly);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code frameTuple2DReadOnly} and the z-component to the given {@code z}.
    * 
    * @param frameTuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @param z the new z-coordinate for this tuple.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   public void set(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
   {
      checkReferenceFrameMatch(frameTuple2DReadOnly);
      tuple.set(frameTuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code other}.
    * 
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void set(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.set(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndAbsolute(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndAbsolute(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndNegate(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndScale(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndScale(scaleFactor, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMax(double)}.
    *
    * @param max the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMax(double max, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMax(max, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMin(double)}.
    *
    * @param min the minimum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMin(double min, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMin(min, other);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMinMax(double, double)}.
    *
    * @param min the minimum value for each component of this frame tuple.
    * @param max the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void setAndClipToMinMax(double min, double max, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMinMax(min, max, other);
   }

   /**
    * Sets this frame tuple's components to {@code x}, {@code y}, and {@code z} and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param x the new value for the x-component of this tuple.
    * @param y the new value for the y-component of this tuple.
    * @param z the new value for the z-component of this tuple.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(x, y, z);
   }

   /**
    * Sets this frame tuple x and y components to {@code tuple2DReadOnly}'s x and y components, the
    * z-component to the given {@code z}, and sets the current frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param tuple2DReadOnly the tuple to copy the values from. Not modified.
    * @param z the new z-coordinate for this frame tuple.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly, double z)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param tuple3DReadOnly the tuple to copy the values from. Not modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      this.referenceFrame = referenceFrame;
      this.tuple.set(tuple3DReadOnly);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param tupleArray the array containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given array
    * {@code tupleArray} and sets this tuple frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startIndex, tupleArray);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from its first row index and sets this tuple frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleDenseMatrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from {@code startRow} and sets this tuple frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, tupleDenseMatrix);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given matrix
    * starting to read from {@code startRow} at the column index {@code column} and sets this tuple
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this tuple.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column the column index to read in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, column, tupleDenseMatrix);
   }

   /**
    * Sets this frame tuple x and y components to {@code tuple2DReadOnly}'s x and y components, the
    * z-component to the given {@code z}, and sets the current frame to
    * {@code frameTuple2DReadOnly.getReferenceFrame()}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the values and reference frame from. Not
    *           modified.
    * @param z the new z-coordinate for this frame tuple.
    */
   public final void setIncludingFrame(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
   {
      referenceFrame = frameTuple2DReadOnly.getReferenceFrame();
      tuple.set(frameTuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   public final void setIncludingFrame(FrameTuple3DReadOnly other)
   {
      referenceFrame = other.getReferenceFrame();
      tuple.set(other);
   }

   /**
    * Adds the given frame tuple to this frame tuple.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other frame tuple to add to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void add(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.add(other);
   }

   /**
    * Sets this frame tuple to the sum of the two given frame tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void add(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param tuple2 the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1, tuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void add(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(tuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple to this frame tuple.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other frame tuple to subtract to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void sub(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.sub(other);
   }

   /**
    * Sets this frame tuple to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void sub(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2 the second tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void sub(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and adds {@code other}.
    * <p>
    * this = scalar * this + other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleAdd(scaleFactor, other);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 + frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 + frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleAdd(double scaleFactor, Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 + tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param tuple2 the second tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scaleFactor, frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and subtracts {@code other}.
    * <p>
    * this = scalar * this - other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleSub(scaleFactor, other);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 - frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param tuple1 the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scaleFactor, Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scaleFactor, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 - tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Performs a linear interpolation from this frame tuple to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other frame tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this frame tuple, while a value of 1 is equivalent to setting this frame
    *           tuple to {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public final void interpolate(FrameTuple3DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      tuple.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code frameTuple1}, while a value of 1 is equivalent to setting
    *           this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same frame as {@code this}.
    */
   public final void interpolate(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code tuple1}, while a value of 1 is equivalent to setting this
    *           frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(tuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param tuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame tuple to {@code frameTuple1}, while a value of 1 is equivalent to setting
    *           this frame tuple to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            frame as {@code this}.
    */
   public final void interpolate(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.interpolate(frameTuple1, tuple2, alpha);
   }

   /** {@inheritDoc} */
   @Override
   public final double getX()
   {
      return tuple.getX();
   }

   /** {@inheritDoc} */
   @Override
   public final double getY()
   {
      return tuple.getY();
   }

   /** {@inheritDoc} */
   @Override
   public final double getZ()
   {
      return tuple.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public final void get(Tuple3DBasics tuple3dToPack)
   {
      tuple3dToPack.set(tuple);
   }
}
