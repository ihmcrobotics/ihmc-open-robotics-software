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
 * One of the main goals of this class is to check, at runtime, that operations on tuples occur
 * within the same Frame. This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public abstract class FrameTuple3D<S extends FrameTuple3D<S, T>, T extends Tuple3DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple3DReadOnly, Tuple3DBasics, Serializable
{
   private static final long serialVersionUID = 3894861900288076730L;

   protected final T tuple;

   public FrameTuple3D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   public final void set(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.set(other);
   }

   public final void setAndAbsolute(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndAbsolute(other);
   }

   public final void setAndNegate(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
   }

   public final void setAndScale(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndScale(scaleFactor, other);
   }

   public void setAndClipToMin(double min, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMin(min, other);
   }

   public void setAndClipToMax(double max, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMax(max, other);
   }

   public void setAndClipToMinMax(double min, double max, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMinMax(min, max, other);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple)
   {
      this.referenceFrame = referenceFrame;
      this.tuple.set(tuple);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple, double z)
   {
      this.referenceFrame = referenceFrame;
      set(tuple, z);
   }

   public void setIncludingFrame(FrameTuple2DReadOnly frameTuple2d, double z)
   {
      referenceFrame = frameTuple2d.getReferenceFrame();
      set(frameTuple2d, z);
   }

   public final void setIncludingFrame(FrameTuple3DReadOnly other)
   {
      referenceFrame = other.getReferenceFrame();
      tuple.set(other);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(x, y, z);
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

   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleAdd(scaleFactor, other);
   }

   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly tuple1, FrameTuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple1);
      checkReferenceFrameMatch(tuple2);
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   public final void scaleAdd(double scaleFactor, Tuple3DReadOnly tuple1, FrameTuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple2);
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple1);
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleSub(scaleFactor, other);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then subs
    * frameTuple2 (this = scaleFactor * frameTuple1 - frameTuple2). Checks if reference frames
    * match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @param frameTuple2 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
   }

   public final void scaleSub(double scaleFactor, Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
   }

   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    * 
    * @param other the other Tuple3d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.add(other);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this =
    * frameTuple1 + frameTuple2).
    * 
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @param frameTuple2 the second FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   public final void add(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1, tuple2);
   }

   public final void add(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -=
    * frameTuple1).
    * 
    * @param other the first FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.sub(other);
   }

   /**
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this =
    * frameTuple1 - frameTuple2).
    * 
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @param frameTuple2 the second FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public final void sub(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public final void sub(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public final void interpolate(FrameTuple3DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      tuple.interpolate(other, alpha);
   }

   /**
    * Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:
    * this = (1-alpha) * tuple1 + alpha * tuple2.
    * 
    * @param t1 the first tuple
    * @param t2 the second tuple
    * @param alpha the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   public final void interpolate(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   public final void interpolate(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   @Override
   public final void setX(double x)
   {
      tuple.setX(x);
   }

   @Override
   public final void setY(double y)
   {
      tuple.setY(y);
   }

   @Override
   public final void setZ(double z)
   {
      tuple.setZ(z);
   }

   public void set(FrameTuple2DReadOnly frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      tuple.set(frameTuple2d);
   }

   public void set(FrameTuple2DReadOnly frameTuple2d, double z)
   {
      checkReferenceFrameMatch(frameTuple2d);
      tuple.set(frameTuple2d, z);
   }

   @Override
   public final double getX()
   {
      return tuple.getX();
   }

   @Override
   public final double getY()
   {
      return tuple.getY();
   }

   @Override
   public final double getZ()
   {
      return tuple.getZ();
   }

   @Override
   public final void get(Tuple3DBasics tuple3dToPack)
   {
      tuple3dToPack.set(tuple);
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }
}
