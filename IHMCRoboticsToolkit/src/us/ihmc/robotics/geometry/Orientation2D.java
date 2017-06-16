package us.ihmc.robotics.geometry;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * A {@code Orientation2D} represents an orientation in the XY-plane, i.e. the yaw angle about the
 * z-axis.
 */
public class Orientation2D implements GeometryObject<Orientation2D>
{
   /** The angle in radians about the z-axis. */
   private double yaw = 0.0;

   /** Vector used to transform {@code this} in {@link #applyTransform(Transform)}. */
   private final Vector2D xVector = new Vector2D(1.0, 0.0);

   /**
    * Creates a new orientation 2D initialized with its yaw angle to zero.
    */
   public Orientation2D()
   {
      yaw = 0.0;
   }

   /**
    * Creates a new orientation 2D and initializes its yaw angle to the given {@code yaw}.
    * 
    * @param yaw the yaw angle used to initialize this.
    */
   public Orientation2D(double yaw)
   {
      setYaw(yaw);
   }

   /**
    * Creates a new orientation 2D and initializes it to {@code other}.
    * 
    * @param other the other orientation 2D used to initialize this. Not modified.
    */
   public Orientation2D(Orientation2D other)
   {
      set(other);
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    * 
    * @param other the other orientation 2D. Not modified.
    */
   @Override
   public void set(Orientation2D other)
   {
      yaw = other.yaw;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(yaw);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      yaw = Double.NaN;
   }

   /**
    * Sets the yaw angle of this orientation 2D to zero.
    */
   @Override
   public void setToZero()
   {
      yaw = 0.0;
   }

   /**
    * Sets the yaw angle of this orientation 2D.
    * <p>
    * Note that the argument is trimmed to be contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param yaw the new yaw angle value in radians.
    */
   public void setYaw(double yaw)
   {
      this.yaw = AngleTools.trimAngleMinusPiToPi(yaw);
   }

   /**
    * Returns the current yaw angle of this orientation 2D.
    * 
    * @return the angle value in radians.
    */
   public double getYaw()
   {
      return yaw;
   }

   /**
    * Adds the given {@code yaw} angle to this orientation 2D:<br>
    * {@code this.yaw += yaw}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param yaw the angle to add to this.
    */
   public void add(double yaw)
   {
      add(this.yaw, yaw);
   }

   /**
    * Adds the other orientation 2D to this:<br>
    * {@code this += other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param other the other orientation 2D to add to this. Not modified.
    */
   public void add(Orientation2D other)
   {
      add(other.yaw);
   }

   /**
    * Sets this orientation 2D to the sum of the two given yaw angles:<br>
    * {@code this.yaw = yaw1 + yaw2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param yaw1 the first yaw angle.
    * @param yaw2 the second yaw angle.
    */
   public void add(double yaw1, double yaw2)
   {
      this.yaw = AngleTools.trimAngleMinusPiToPi(yaw1 + yaw2);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void add(Orientation2D orientation1, Orientation2D orientation2)
   {
      add(orientation1.yaw, orientation2.yaw);
   }

   /**
    * Subtracts the given {@code yaw} angle from this orientation 2D:<br>
    * {@code this.yaw -= yaw}
    * 
    * @param yaw the angle to subtract.
    */
   public void sub(double yaw)
   {
      sub(this.yaw, yaw);
   }

   /**
    * Subtracts the other orientation 2D from this orientation 2D:<br>
    * {@code this -= other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param other the other orientation 2D to subtract. Not modified.
    */
   public void sub(Orientation2D other)
   {
      sub(other.yaw);
   }

   /**
    * Sets this orientation 2D to the difference of the two given yaw angles:<br>
    * {@code this.yaw = yaw1 - yaw2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param yaw1 the first yaw angle.
    * @param yaw2 the second yaw angle.
    */
   public void sub(double yaw1, double yaw2)
   {
      this.yaw = AngleTools.computeAngleDifferenceMinusPiToPi(yaw1, yaw2);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    * 
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void sub(Orientation2D orientation1, Orientation2D orientation2)
   {
      sub(orientation1.yaw, orientation2.yaw);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    * 
    * @param other the other orientation 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   public void interpolate(Orientation2D other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code orientation1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code orientation2}.
    */
   public void interpolate(Orientation2D orientation1, Orientation2D orientation2, double alpha)
   {
      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(orientation2.yaw, orientation1.yaw);
      add(orientation1.yaw, alpha * deltaYaw);
   }

   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by
    * this.
    * 
    * <pre>
    * tupleToTransform = / cos(yaw) -sin(yaw) \ * tupleToTransform
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    * 
    * @param tupleToTransform the tuple to transform. Modified.
    */
   public void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    * 
    * <pre>
    * tupleTransformed = / cos(yaw) -sin(yaw) \ * tupleOriginal
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    * 
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(yaw, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by
    * this.
    * 
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleToTransform = | sin(yaw)  cos(yaw) 0 | * tupleToTransform
    *                    \    0         0     1 /
    * </pre>
    * 
    * @param tupleToTransform the tuple to transform. Modified.
    */
   public void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    * 
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    * 
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(yaw, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation
    * about the z-axis described by this.
    * 
    * <pre>
    * tupleToTransform = / cos(-yaw) -sin(-yaw) \ * tupleToTransform
    *                    \ sin(-yaw)  cos(-yaw) /
    * </pre>
    * 
    * @param tupleToTransform the tuple to transform. Modified.
    */
   public void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleOriginal} by the rotation about
    * the z-axis described by this and stores the result in {@code tupleTransformed}.
    * 
    * <pre>
    * tupleTransformed = / cos(-yaw) -sin(-yaw) \ * tupleOriginal
    *                    \ sin(-yaw)  cos(-yaw) /
    * </pre>
    * 
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(-yaw, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation
    * about the z-axis described by this.
    * 
    * <pre>
    *                    / cos(-yaw) -sin(-yaw) 0 \
    * tupleToTransform = | sin(-yaw)  cos(-yaw) 0 | * tupleToTransform
    *                    \     0          0     1 /
    * </pre>
    * 
    * @param tupleToTransform the tuple to transform. Modified.
    */
   public void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleOriginal} by the rotation about
    * the z-axis described by this and stores the result in {@code tupleTransformed}.
    * 
    * <pre>
    *                    / cos(-yaw) -sin(-yaw) 0 \
    * tupleTransformed = | sin(-yaw)  cos(-yaw) 0 | * tupleOriginal
    *                    \     0          0     1 /
    * </pre>
    * 
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(-yaw, tupleOriginal, tupleTransformed);
   }

   /**
    * Computes the distance between {@code this} and {@code other} as the absolute difference in
    * angle:<br>
    * {@code distance = Math.abs(this.yaw - other.yaw)}
    * 
    * @param other the other orientation 2D. Not modified.
    * @return the distance between {@code this} and {@code other} contained in [0, <i>pi</pi>].
    */
   public double distance(Orientation2D other)
   {
      return Math.abs(difference(other));
   }

   /**
    * Computes and returns the difference between {@code this} and {@code other}:<br>
    * {@code distance = this.yaw - other.yaw}
    * 
    * @param other the other orientation 2D. Not modified.
    * @return the difference between {@code this} and {@code other} contained in [-<i>pi</i>, <i>pi</pi>].
    */
   public double difference(Orientation2D other)
   {
      return AngleTools.computeAngleDifferenceMinusPiToPi(yaw, other.yaw);
   }

   /**
    * Transforms this orientation 2D by the given {@code transform}.
    * <p>
    * This is equivalent to extracting the yaw rotation part from the given transform and adding it
    * to this.
    * </p>
    *
    * @param transform the geometric transform to apply on this orientation 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      xVector.set(1.0, 0.0);
      transform.transform(xVector);
      double deltaYaw = Math.atan2(xVector.getY(), xVector.getX());

      if (Double.isNaN(deltaYaw) || Double.isInfinite(deltaYaw))
         deltaYaw = 0.0;

      add(deltaYaw);
   }

   /**
    * Tests if the yaw angle of this orientation is equal to an {@code epsilon} to the yaw of
    * {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two orientations are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Orientation2D other, double epsilon)
   {
      return MathTools.epsilonEquals(yaw, other.yaw, epsilon);
   }

   /**
    * Tests if this orientation 2D is exactly equal to {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other the other orientation 2D to compare against this. Not modified.
    * @return {@code true} if the two orientations are exactly equal, {@code false} otherwise.
    */
   public boolean equals(Orientation2D other)
   {
      if (other == null)
         return false;
      else
         return yaw == other.yaw;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Orientation2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Orientation2D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this orientation 2D as follows:<br>
    * (0.123 )
    *
    * @return the {@code String} representing this orientation 2D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", yaw);
   }
}
