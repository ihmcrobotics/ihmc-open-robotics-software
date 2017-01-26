package us.ihmc.robotics.geometry;

import java.io.Serializable;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameTuple2d<S extends FrameTuple2d<S, T>, T extends Tuple2d & GeometryObject<T>> extends AbstractFrameObject<S, T> implements Serializable
{
   private static final long serialVersionUID = 6275308250031489785L;

   private static final boolean DEBUG = false;

   protected final T tuple;
   protected String name;
   
   public FrameTuple2d(ReferenceFrame referenceFrame, T tuple, String name)
   {
      super(referenceFrame, tuple);
      
      if (DEBUG)
      {
         if (referenceFrame == null)
         {
            String errorMsg = "FrameTuple2d: created a " + getClass().getSimpleName() + " with a null reference frame.";
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

   public final void set(double x, double y)
   {
      tuple.setX(x);
      tuple.setY(y);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y)
   {
      this.referenceFrame = referenceFrame;
      set(x, y);
   }

   public final void set(Tuple2d tuple)
   {
      this.tuple.set(tuple);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2d tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   /**
    * 
    * @throws ReferenceFrameMismatchException
    */
   public final void set(FrameTuple2d<?, ?> frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      set(frameTuple2d.tuple);
   }

   /**
    * Set this frameTuple2d using the x and y coordinate the frameTuple passed in.
    * @throws ReferenceFrameMismatchException
    */
   public final void setByProjectionOntoXYPlane(FrameTuple<?, ?> frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      set(frameTuple.getX(), frameTuple.getY());
   }

   /**
    * Set this frameTuple2d using the x and y coordinate the frameTuple passed in.
    * @throws ReferenceFrameMismatchException
    */
   public final void setByProjectionOntoXYPlaneIncludingFrame(FrameTuple<?, ?> frameTuple)
   {
      setIncludingFrame(frameTuple.getReferenceFrame(), frameTuple.getX(), frameTuple.getY());
   }

   public final void setIncludingFrame(FrameTuple2d<?, ?> frameTuple2d)
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

   public final void scale(double scaleFactor)
   {
      tuple.scale(scaleFactor);
   }

   public final void scale(double scaleXFactor, double scaleYFactor)
   {
      tuple.setX(tuple.getX() * scaleXFactor);
      tuple.setY(tuple.getY() * scaleYFactor);
   }

   public final double getX()
   {
      return tuple.getX();
   }

   public final double getY()
   {
      return tuple.getY();
   }

   public double distanceFromZero()
   {
      return Math.sqrt(tuple.getX() * tuple.getX() + tuple.getY() * tuple.getY());
   }

   /**
    * Returns a Point2d copy of the tuple in this FrameTuple.
    *
    * @return Point2d
    */
   public final Point2d getPointCopy()
   {
      return new Point2d(tuple);
   }

   /**
    * Returns a Vector2d copy of the tuple in this FrameTuple.
    *
    * @return Vector2d
    */
   public final Vector2d getVectorCopy()
   {
      return new Vector2d(this.tuple);
   }

   public final void get(Tuple2d tuple2dToPack)
   {
      tuple2dToPack.set(tuple);
   }

   /**
    * Pack this tuple2d in tuple3dToPack and tuple3dToPack.z = 0.0.
    * @param tuple3dToPack {@code Tuple3d}
    */
   public final void get(Tuple3d tuple3dToPack)
   {
      tuple3dToPack.set(tuple.getX(), tuple.getY(), 0.0);
   }

   /**
    * Pack this tuple2d in tuple3fToPack and tuple3fToPack.z = 0.0.
    * @param tuple3fToPack {@code Tuple3f}
    */
   public final void get(Tuple3f tuple3fToPack)
   {
      tuple3fToPack.set((float) tuple.getX(), (float) tuple.getY(), 0.0f);
   }

   public final void setToZero()
   {
      tuple.set(0.0, 0.0);
   }

   public final void setToZero(ReferenceFrame referenceFrame)
   {
      setToZero();
      this.referenceFrame = referenceFrame;
   }

   public final void setToNaN()
   {
      this.tuple.set(Double.NaN, Double.NaN);
   }

   public final void setToNaN(ReferenceFrame referenceFrame)
   {
      this.tuple.set(Double.NaN, Double.NaN);
      this.referenceFrame = referenceFrame;
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }

   public final boolean containsNaN()
   {
      return Double.isNaN(tuple.getX()) || Double.isNaN(tuple.getY());
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 (this = scaleFactor * tuple1).
    *
    * @param scaleFactor double
    * @param frameTuple1 Tuple2d
    */
   public final void scale(double scaleFactor, Tuple2d tuple1)
   {
      tuple.scale(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds tuple2 (this = scaleFactor * tuple1 + tuple2).
    *
    * @param scaleFactor double
    * @param frameTuple1 Tuple2d
    * @param frameTuple2 Tuple2d
    */
   public final void scaleAdd(double scaleFactor, Tuple2d tuple1, Tuple2d tuple2)
   {
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds the scalar multiplication of tuple2 (this = scaleFactor1 * tuple1 + scaleFactor2 * tuple2).
    *
    * @param scaleFactor1 double
    * @param frameTuple1 Tuple2d
    * @param scaleFactor2 double
    * @param frameTuple2 Tuple2d
    */
   public final void scaleAdd(double scaleFactor1, Tuple2d tuple1, double scaleFactor2, Tuple2d tuple2)
   {
      tuple.setX(scaleFactor1 * tuple1.getX() + scaleFactor2 * tuple2.getX());
      tuple.setY(scaleFactor1 * tuple1.getY() + scaleFactor2 * tuple2.getY());
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds tuple1 (this = scaleFactor * this + tuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 Tuple2d
    */
   public final void scaleAdd(double scaleFactor, Tuple2d tuple1)
   {
      tuple.scaleAdd(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 (this = scaleFactor * frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scale(double scaleFactor, FrameTuple2d<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scale(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then adds frameTuple2 (this = scaleFactor * frameTuple1 + frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @param frameTuple2 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      scaleAdd(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1 (this = scaleFactor * this + frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2d<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scaleAdd(scaleFactor, frameTuple1.tuple);
   }

   public final void add(double dx, double dy)
   {
      tuple.setX(tuple.getX() + dx);
      tuple.setY(tuple.getY() + dy);
   }

   /**  
    * Sets the value of this tuple to the sum of itself and tuple1.
    * @param tuple1 the other Tuple2d
    */
   public final void add(Tuple2d tuple1)
   {
      tuple.add(tuple1);
   }

   /**
    * Sets the value of this tuple to the sum of tuple1 and tuple2 (this = tuple1 + tuple2).
    * @param tuple1 the first Tuple2d
    * @param tuple2 the second Tuple2d
    */
   public final void add(Tuple2d tuple1, Tuple2d tuple2)
   {
      tuple.add(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    * @param frameTuple1 the other Tuple2d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2d<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      add(frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this = frameTuple1 + frameTuple2).
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      add(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**  
    * Sets the value of this tuple to the difference of itself and tuple1 (this -= tuple1).
    * @param tuple1 the other Tuple2d
    */
   public final void sub(Tuple2d tuple1)
   {
      tuple.sub(tuple1);
   }
   
   public final void sub(double dx, double dy)
   {
      tuple.setX(tuple.getX() - dx);
      tuple.setY(tuple.getY() - dy);
   }

   /**  
    * Sets the value of this tuple to the difference of tuple1 and tuple2 (this = tuple1 - tuple2).
    * @param tuple1 the first Tuple2d
    * @param tuple2 the second Tuple2d
    */
   public final void sub(Tuple2d tuple1, Tuple2d tuple2)
   {
      tuple.sub(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -= frameTuple1).
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2d<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      sub(frameTuple1.tuple);
   }

   /**  
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this = frameTuple1 - frameTuple2).
    * @param frameTuple1 the first FrameTuple2d<?, ?>
    * @param frameTuple2 the second FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      sub(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
     *  Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:  this = (1-alpha) * tuple1 + alpha * tuple2.
     *  @param t1  the first tuple
     *  @param t2  the second tuple  
     *  @param alpha  the alpha interpolation parameter  
    */
   public final void interpolate(Tuple2d tuple1, Tuple2d tuple2, double alpha)
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
   public final void interpolate(FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2, double alpha)
   {
      frameTuple1.checkReferenceFrameMatch(frameTuple2);

      interpolate(frameTuple1.tuple, frameTuple2.tuple, alpha);
      referenceFrame = frameTuple1.getReferenceFrame();
   }

   public final void clipToMinMax(double minValue, double maxValue)
   {
      this.tuple.clamp(minValue, maxValue);
   }

   public final void negate()
   {
      tuple.negate();
   }

   /**
     * Returns true if the L-infinite distance between this tuple and tuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2)].
    * @param tuple1 Tuple2d
    * @param threshold double
    */
   public final boolean epsilonEquals(Tuple2d tuple1, double threshold)
   {
      return tuple.epsilonEquals(tuple1, threshold);
   }

   /**
     * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2)].
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple2d<?, ?> frameTuple1, double threshold)
   {
      checkReferenceFrameMatch(frameTuple1);

      return epsilonEquals(frameTuple1.tuple, threshold);
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

//   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);
//
//   public abstract FrameTuple2d<?, ?> changeFrameUsingTransformCopy(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);
//
//   public abstract void applyTransform(RigidBodyTransform transform);
//
//   public abstract FrameTuple2d<?, ?> applyTransformCopy(RigidBodyTransform transform);
//
//   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public final double[] toArray()
   {
      return new double[] { tuple.getX(), tuple.getY() };
   }

   /**
    * toString
    * <p/>
    * String representation of a FrameVector (x,y):reference frame name
    *
    * @return String
    */
   @Override
   public final String toString()
   {
      return "" + tuple + "-" + referenceFrame;
   }
}
