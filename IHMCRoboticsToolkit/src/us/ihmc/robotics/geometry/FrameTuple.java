package us.ihmc.robotics.geometry;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * One of the main goals of this class is to check, at runtime, that operations on tuples occur within the same Frame.
 * This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public abstract class FrameTuple<S extends FrameTuple<S, T>, T extends Tuple3DBasics & GeometryObject<T>> extends AbstractFrameObject<S, T> implements Serializable
{
   private static final long serialVersionUID = 3894861900288076730L;

   private static final boolean DEBUG = false;

   protected final T tuple;
   protected String name;

   public FrameTuple(ReferenceFrame referenceFrame, T tuple, String name)
   {
      super(referenceFrame, tuple);

      if (DEBUG)
      {
         if (referenceFrame == null)
         {
            String errorMsg = "FrameTuple: created a " + "className" + " with a null reference frame.";
            System.err.println(errorMsg);
         }
      }

      this.tuple = getGeometryObject();
      this.name = name;
   }

   public final void setName(String name)
   {
      this.name = name;
   }

   public final String getName()
   {
      return name;
   }

   public final void set(double x, double y, double z)
   {
      tuple.setX(x);
      tuple.setY(y);
      tuple.setZ(z);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this.referenceFrame = referenceFrame;
      set(x, y, z);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleArray);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startIndex, tupleArray);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(tupleDenseMatrix);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, tupleDenseMatrix);
   }
   
   public final void setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
   {
      this.referenceFrame = referenceFrame;
      tuple.set(startRow, column, tupleDenseMatrix);
   }

   public final void set(FrameTuple<?, ?> frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      this.tuple.set(frameTuple.tuple);
   }

   @Override
   public final void set(Tuple3DBasics tuple)
   {
      this.tuple.set(tuple);
   }

   public final void set(Tuple3DReadOnly tuple)
   {
      this.tuple.set(tuple);
   }

   @Override
   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DBasics tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   /**
    * Set the x and y components of this frameTuple to tuple2d.x and tuple2d.y respectively, and sets the z component to zero.
    * @param tuple2d
    */
   public final void setXYIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple)
   {
      this.referenceFrame = referenceFrame;
      setXY(tuple);
   }

   /**
    * Set the x and y components of this frameTuple to frameTuple2d.x and frameTuple2d.y respectively, and sets the z component to zero.
    * Changes the referenceFrame of this frameTuple to frameTuple2d.getReferenceFrame().
    * @param frameTuple2d
    */
   public void setXYIncludingFrame(FrameTuple2d<?, ?> frameTuple2d)
   {
      this.referenceFrame = frameTuple2d.getReferenceFrame();
      setXY(frameTuple2d);
   }

   public final void setIncludingFrame(FrameTuple<?, ?> frameTuple)
   {
      setIncludingFrame(frameTuple.referenceFrame, frameTuple.tuple);
   }

   public final void setX(double x)
   {
      tuple.setX(x);
   }

   public final void setY(double y)
   {
      tuple.setY(y);
   }

   public final void setZ(double z)
   {
      tuple.setZ(z);
   }

   public final void set(Direction direction, double value)
   {
      Direction.set(tuple, direction, value);
   }

   /**
    * Set the x and y components of this frameTuple to tuple2d.x and tuple2d.y respectively, and sets the z component to zero.
    * @param tuple2d
    */
   public void setXY(Tuple2DReadOnly tuple2d)
   {
      this.tuple.setX(tuple2d.getX());
      this.tuple.setY(tuple2d.getY());
      this.tuple.setZ(0.0);
   }

   /**
    * Set the x and y components of this frameTuple to frameTuple2d.x and frameTuple2d.y respectively, and sets the z component to zero.
    * @param frameTuple2d
    * @throws ReferenceFrameMismatchException
    */
   public void setXY(FrameTuple2d<?, ?> frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      setXY(frameTuple2d.tuple);
   }

   public final double get(Direction direction)
   {
      return Direction.get(tuple, direction);
   }

   public final void scale(double scaleFactor)
   {
      tuple.scale(scaleFactor);
   }

   public final void scale(double scaleXFactor, double scaleYFactor, double scaleZFactor)
   {
      tuple.setX(tuple.getX() * scaleXFactor);
      tuple.setY(tuple.getY() * scaleYFactor);
      tuple.setZ(tuple.getZ() * scaleZFactor);
   }

   public final double getX()
   {
      return tuple.getX();
   }

   public final double getY()
   {
      return tuple.getY();
   }

   public final double getZ()
   {
      return tuple.getZ();
   }

   /**
    * Returns a Point3D copy of the tuple in this FrameTuple.
    *
    * @return Point3D
    */
   public final Point3D getPointCopy()
   {
      return new Point3D(tuple);
   }

   /**
    * Returns a Vector3d copy of the tuple in this FrameTuple.
    *
    * @return Vector3d
    */
   public final Vector3D getVectorCopy()
   {
      return new Vector3D(this.tuple);
   }

   @Override
   public final void get(Tuple3DBasics tuple3dToPack)
   {
      tuple3dToPack.set(tuple);
   }

   @Override
   public final void setToZero()
   {
      tuple.setToZero();
   }

   @Override
   public final void setToZero(ReferenceFrame referenceFrame)
   {
      setToZero();
      this.referenceFrame = referenceFrame;
   }

   @Override
   public final void setToNaN()
   {
      this.tuple.setToNaN();
   }

   @Override
   public final void setToNaN(ReferenceFrame referenceFrame)
   {
      this.tuple.setToNaN();
      this.referenceFrame = referenceFrame;
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }

   @Override
   public final boolean containsNaN()
   {
      return tuple.containsNaN();
   }

   public final boolean containsInfinity()
   {
      return Double.isInfinite(tuple.getX()) || Double.isInfinite(tuple.getY()) || Double.isInfinite(tuple.getZ());
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 (this = scaleFactor * tuple1).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    */
   public final void setAndScale(double scaleFactor, Tuple3DReadOnly tuple1)
   {
      tuple.setAndScale(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds tuple2 (this = scaleFactor * tuple1 + tuple2).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    * @param tuple2 Tuple3d
    */
   public final void scaleAdd(double scaleFactor, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds the scalar multiplication of tuple2 (this = scaleFactor1 * tuple1 + scaleFactor2 * tuple2).
    *
    * @param scaleFactor1 double
    * @param frameTuple1 Tuple3d
    * @param scaleFactor2 double
    * @param frameTuple2 Tuple3d
    */
   public final void scaleAdd(double scaleFactor1, Tuple3DReadOnly tuple1, double scaleFactor2, Tuple3DReadOnly tuple2)
   {
      tuple.setX(scaleFactor1 * tuple1.getX() + scaleFactor2 * tuple2.getX());
      tuple.setY(scaleFactor1 * tuple1.getY() + scaleFactor2 * tuple2.getY());
      tuple.setZ(scaleFactor1 * tuple1.getZ() + scaleFactor2 * tuple2.getZ());
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds tuple1 (this = scaleFactor * this + tuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    */
   public final void scaleAdd(double scaleFactor, Tuple3DReadOnly tuple1)
   {
      tuple.scaleAdd(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 (this = scaleFactor * frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scale(double scaleFactor, FrameTuple<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      setAndScale(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1 (this = scaleFactor * this + frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scaleAdd(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then adds frameTuple2 (this = scaleFactor * frameTuple1 + frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @param frameTuple2 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      scaleAdd(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then subs frameTuple2 (this = scaleFactor * frameTuple1 - frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @param frameTuple2 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleSub(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);

      set(frameTuple1);
      scale(scaleFactor);
      sub(frameTuple2);
   }

   /**  
    * Sets the value of this tuple to the sum of itself and tuple1.
    * @param tuple1 the other Tuple3d
    */
   public final void add(Tuple3DReadOnly tuple1)
   {
      tuple.add(tuple1);
   }

   /**  
    * Sets the value of this tuple to the sum of itself and tuple1.
    * @param tuple1 the other Tuple3d
    */
   public final void add(double x, double y, double z)
   {
      tuple.setX(tuple.getX() + x);
      tuple.setY(tuple.getY() + y);
      tuple.setZ(tuple.getZ() + z);
   }

   /**
    * Sets the value of this tuple to the sum of tuple1 and tuple2 (this = tuple1 + tuple2).
    * @param tuple1 the first Tuple3d
    * @param tuple2 the second Tuple3d
    */
   public final void add(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      tuple.add(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    * @param frameTuple1 the other Tuple3d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      add(frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this = frameTuple1 + frameTuple2).
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @param frameTuple2 the second FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      add(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**  
    * Sets the value of this tuple to the difference of itself and tuple1 (this -= tuple1).
    * @param tuple1 the other Tuple3d
    */
   public final void sub(double x, double y, double z)
   {
      tuple.setX(tuple.getX() - x);
      tuple.setY(tuple.getY() - y);
      tuple.setZ(tuple.getZ() - z);
   }

   /**  
    * Sets the value of this tuple to the difference of itself and tuple1 (this -= tuple1).
    * @param tuple1 the other Tuple3d
    */
   public final void sub(Tuple3DReadOnly tuple1)
   {
      tuple.sub(tuple1);
   }

   /**  
    * Sets the value of this tuple to the difference of tuple1 and tuple2 (this = tuple1 - tuple2).
    * @param tuple1 the first Tuple3d
    * @param tuple2 the second Tuple3d
    */
   public final void sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      tuple.sub(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -= frameTuple1).
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      sub(frameTuple1.tuple);
   }

   /**  
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this = frameTuple1 - frameTuple2).
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @param frameTuple2 the second FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      sub(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 minus tuple2 (this = scaleFactor * ( tuple1 - tuple2 ) ).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    * @param tuple2 Tuple3d
    */
   public final void subAndScale(double scaleFactor, Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      sub(tuple1, tuple2);
      scale(scaleFactor);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of frameTuple1 minus frameTuple2 (this = scaleFactor * ( frameTuple1 - frameTuple2 ) ).
    *
    * @param scaleFactor double
    * @param frameTuple1 the first FrameTuple<?, ?>
    * @param frameTuple2 the second FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void subAndScale(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      subAndScale(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
     *  Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:  this = (1-alpha) * tuple1 + alpha * tuple2.
     *  @param t1  the first tuple
     *  @param t2  the second tuple  
     *  @param alpha  the alpha interpolation parameter  
    */
   public final void interpolate(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double alpha)
   {
      tuple.interpolate(tuple1, tuple2, alpha);
   }

   /**
     *  Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:  this = (1-alpha) * tuple1 + alpha * tuple2.
     *  @param t1  the first tuple
     *  @param t2  the second tuple  
     *  @param alpha  the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2, double alpha)
   {
      frameTuple1.checkReferenceFrameMatch(frameTuple2);

      interpolate(frameTuple1.tuple, frameTuple2.tuple, alpha);
      referenceFrame = frameTuple1.getReferenceFrame();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * its first row index.
    *
    * @param tupleMatrixToPack the array in which this tuple is frame stored. Modified.
    */
   public final void get(DenseMatrix64F tupleMatrixToPack)
   {
      tuple.get(tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * {@code startRow}.
    *
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param tupleMatrixToPack the column vector in which this frame tuple is stored. Modified.
    */
   public final void get(int startRow, DenseMatrix64F tupleMatrixToPack)
   {
      tuple.get(startRow, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * {@code startRow} at the column index {@code column}.
    *
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param column the column index to write in the dense-matrix.
    * @param tupleMatrixToPack the matrix in which this frame tuple is stored. Modified.
    */
   public final void get(int startRow, int column, DenseMatrix64F tupleMatrixToPack)
   {
      tuple.get(startRow, column, tupleMatrixToPack);
   }

   public final void clipToMinMax(double minValue, double maxValue)
   {
      this.tuple.clipToMinMax(minValue, maxValue);
   }

   public final void negate()
   {
      tuple.negate();
   }

   public final void absolute()
   {
      tuple.absolute();
   }

   /**
     * Returns true if the L-infinite distance between this tuple and tuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-z2)].
    * @param tuple1 Tuple3d
    * @param threshold double
    */
   public final boolean epsilonEquals(Tuple3DReadOnly tuple1, double threshold)
   {
      if (tuple1 == null)
      {
         return false;
      }

      return tuple.epsilonEquals(tuple1, threshold);
   }

   /**
     * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-z2)].
    * @param frameTuple1 FrameTuple<?, ?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple<?, ?> frameTuple1, double threshold)
   {
      if (frameTuple1 == null)
      {
         return false;
      }

      checkReferenceFrameMatch(frameTuple1);

      return epsilonEquals(frameTuple1.tuple, threshold);
   }

   /**
     * Returns true if the L-infinite distance between this tuple and tuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-0)].
    * @param tuple1 Tuple3d
    * @param threshold double
    */
   public final boolean epsilonEquals(Tuple2DReadOnly tuple1, double threshold)
   {
      if (tuple1 == null)
      {
         return false;
      }

      double diff;

      diff = tuple.getX() - tuple1.getX();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      diff = tuple.getY() - tuple1.getY();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      diff = tuple.getZ();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      return true;
   }

   /**
     * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-0)].
    * @param frameTuple1 FrameTuple<?, ?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple2d<?, ?> frameTuple2d, double threshold)
   {
      if (frameTuple2d == null)
      {
         return false;
      }

      checkReferenceFrameMatch(frameTuple2d);

      return epsilonEquals(frameTuple2d.tuple, threshold);
   }

   /**
    * Returns this FrameTuple's ReferenceFrame.
    * @return ReferenceFrame
    */
   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public final double[] toArray()
   {
      return new double[] {tuple.getX(), tuple.getY(), tuple.getZ()};
   }

   /**
    * toString
    * <p/>
    * String representation of a FrameVector (x,y,z):reference frame name
    *
    * @return String
    */
   @Override
   public final String toString()
   {
      return "" + tuple + "-" + referenceFrame.getName();
   }
}
