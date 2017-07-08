package us.ihmc.robotics.geometry;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class FrameTuple2D<S extends FrameTuple2D<S, T>, T extends Tuple2DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple2DReadOnly, Tuple2DBasics, Serializable
{
   private static final long serialVersionUID = 6275308250031489785L;

   protected final T tuple;

   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   public FrameTuple2D(ReferenceFrame referenceFrame, T tuple)
   {
      super(referenceFrame, tuple);
      this.tuple = getGeometryObject();
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y)
   {
      this.referenceFrame = referenceFrame;
      set(x, y);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order from the given array
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
    * Sets this tuple's components {@code x}, {@code y} in order from the given array
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
    * Sets this tuple's components {@code x}, {@code y} in order from the given column vector
    * starting to read from its first row index and sets this tuple frame to {@code referenceFrame}.
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
    * Sets this tuple's components {@code x}, {@code y} in order from the given column vector
    * starting to read from {@code startRow} and sets this tuple frame to {@code referenceFrame}.
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
    * Sets this tuple's components {@code x}, {@code y} in order from the given matrix starting to
    * read from {@code startRow} at the column index {@code column} and sets this tuple frame to
    * {@code referenceFrame}.
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

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   /**
    *
    * @throws ReferenceFrameMismatchException
    */
   public final void set(FrameTuple2DReadOnly frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      tuple.set(frameTuple2d);
   }

   /**
    * Set this frameTuple2d using the x and y coordinate the frameTuple passed in.
    *
    * @throws ReferenceFrameMismatchException
    */
   public final void set(FrameTuple3DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      set(frameTuple.getX(), frameTuple.getY());
   }

   /**
    * Set this frameTuple2d using the x and y coordinate the frameTuple passed in.
    *
    * @throws ReferenceFrameMismatchException
    */
   public final void setIncludingFrame(FrameTuple3DReadOnly frameTuple)
   {
      setIncludingFrame(frameTuple.getReferenceFrame(), frameTuple.getX(), frameTuple.getY());
   }

   public final void setIncludingFrame(FrameTuple2DReadOnly frameTuple2d)
   {
      this.referenceFrame = frameTuple2d.getReferenceFrame();
      tuple.set(frameTuple2d);
   }

   public final void setAndScale(double scaleFactor, FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.setAndScale(scaleFactor, frameTuple1);
   }

   public void setAndClipToMax(double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMax(max, other);
   }

   public void setAndClipToMin(double min, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMin(min, other);
   }

   public void setAndClipToMinMax(double min, double max, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndClipToMinMax(min, max, other);
   }

   public void setAndAbsolute(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndAbsolute(other);
   }

   public void setAndNegate(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
   }

   public final void setX(double x)
   {
      tuple.setX(x);
   }

   public final void setY(double y)
   {
      tuple.setY(y);
   }

   public final double getX()
   {
      return tuple.getX();
   }

   public final double getY()
   {
      return tuple.getY();
   }

   @Override
   public final void get(Tuple2DBasics tuple2dToPack)
   {
      tuple2dToPack.set(tuple);
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then adds
    * frameTuple2 (this = scaleFactor * frameTuple1 + frameTuple2). Checks if reference frames
    * match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @param frameTuple2 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
   }

   public final void scaleAdd(double scaleFactor, Tuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
   }

   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1
    * (this = scaleFactor * this + frameTuple1). Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.scaleAdd(scaleFactor, frameTuple1);
   }

   public void scaleSub(double scalar, FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.scaleSub(scalar, other);
   }

   public void scaleSub(double scalar, FrameTuple2DReadOnly tuple1, FrameTuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple1);
      checkReferenceFrameMatch(tuple2);
      tuple.scaleSub(scalar, tuple1, tuple2);
   }

   public void scaleSub(double scalar, Tuple2DReadOnly tuple1, FrameTuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple2);
      tuple.scaleSub(scalar, tuple1, tuple2);
   }

   public void scaleSub(double scalar, FrameTuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
   {
      checkReferenceFrameMatch(tuple1);
      tuple.scaleSub(scalar, tuple1, tuple2);
   }

   /**
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    *
    * @param frameTuple1 the other Tuple2d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this =
    * frameTuple1 + frameTuple2).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   public final void add(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.add(frameTuple1, frameTuple2);
   }

   public final void add(Tuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -=
    * frameTuple1).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2DReadOnly frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1);
   }

   /**
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this =
    * frameTuple1 - frameTuple2).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public final void sub(Tuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public final void sub(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.sub(frameTuple1, frameTuple2);
   }

   public void interpolate(FrameTuple2DReadOnly other, double alpha)
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
   public final void interpolate(FrameTuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   public final void interpolate(Tuple2DReadOnly frameTuple1, FrameTuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   public final void interpolate(FrameTuple2DReadOnly frameTuple1, Tuple2DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of
       * checking both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, projects into xy plane.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    */
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }
}
