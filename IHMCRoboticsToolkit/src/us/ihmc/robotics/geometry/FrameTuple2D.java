package us.ihmc.robotics.geometry;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public abstract class FrameTuple2D<S extends FrameTuple2D<S, T>, T extends Tuple2DBasics & GeometryObject<T>> extends FrameGeometryObject<S, T>
      implements FrameTuple2DReadOnly, Tuple2DBasics, Serializable
{
   private static final long serialVersionUID = 6275308250031489785L;

   protected final T tuple;

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
   public final void set(FrameTuple2D<?, ?> frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      set(frameTuple2d.tuple);
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

   public final void setIncludingFrame(FrameTuple2D<?, ?> frameTuple2d)
   {
      setIncludingFrame(frameTuple2d.referenceFrame, frameTuple2d.tuple);
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

   /**
    * Returns a Point2d copy of the tuple in this FrameTuple.
    *
    * @return Point2d
    */
   public final Point2D getPointCopy()
   {
      return new Point2D(tuple);
   }

   /**
    * Returns a Vector2d copy of the tuple in this FrameTuple.
    *
    * @return Vector2d
    */
   public final Vector2D getVectorCopy()
   {
      return new Vector2D(this.tuple);
   }

   @Override
   public final void get(Tuple2DBasics tuple2dToPack)
   {
      tuple2dToPack.set(tuple);
   }

   /**
    * Pack this tuple2d in tuple3dToPack and tuple3dToPack.z = 0.0.
    *
    * @param tuple3dToPack {@code Tuple3DBasics}
    */
   public final void get(Tuple3DBasics tuple3dToPack)
   {
      tuple3dToPack.set(tuple.getX(), tuple.getY(), 0.0);
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 (this =
    * scaleFactor * frameTuple1). Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scale(double scaleFactor, FrameTuple2D<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      setAndScale(scaleFactor, frameTuple1.tuple);
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
   public final void scaleAdd(double scaleFactor, FrameTuple2D<?, ?> frameTuple1, FrameTuple2D<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      scaleAdd(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1
    * (this = scaleFactor * this + frameTuple1). Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2D<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scaleAdd(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    *
    * @param frameTuple1 the other Tuple2d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2D<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      add(frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this =
    * frameTuple1 + frameTuple2).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2D<?, ?> frameTuple1, FrameTuple2D<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      add(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -=
    * frameTuple1).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2D<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      sub(frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this =
    * frameTuple1 - frameTuple2).
    *
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2D<?, ?> frameTuple1, FrameTuple2D<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      sub(frameTuple1.tuple, frameTuple2.tuple);
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
   public final void interpolate(FrameTuple2D<?, ?> frameTuple1, FrameTuple2D<?, ?> frameTuple2, double alpha)
   {
      frameTuple1.checkReferenceFrameMatch(frameTuple2);

      interpolate(frameTuple1.tuple, frameTuple2.tuple, alpha);
      referenceFrame = frameTuple1.getReferenceFrame();
   }

   /**
    * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than
    * or equal to the epsilon parameter, otherwise returns false. The L-infinite distance is equal
    * to MAX[abs(x1-x2), abs(y1-y2)].
    *
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple2D<?, ?> frameTuple1, double threshold)
   {
      checkReferenceFrameMatch(frameTuple1);

      return epsilonEquals(frameTuple1.tuple, threshold);
   }
}
