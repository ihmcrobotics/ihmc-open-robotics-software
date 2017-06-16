package us.ihmc.robotics.geometry.transformables;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class Pose implements GeometryObject<Pose>
{
   private final Quaternion orientation = new Quaternion();
   private final Point3D position = new Point3D();

   public Pose()
   {
   }

   public Pose(Pose other)
   {
      orientation.set(other.getOrientation());
      position.set(other.getPosition());
   }

   public Pose(RigidBodyTransform transform)
   {
      transform.get(orientation, position);
   }

   public Pose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   @Override
   public boolean containsNaN()
   {
      return orientation.containsNaN() || position.containsNaN();
   }

   @Override
   public void setToZero()
   {
      orientation.setToZero();
      position.setToZero();
   }

   @Override
   public void setToNaN()
   {
      orientation.setToNaN();
      position.setToNaN();
   }

   public void setX(double x)
   {
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }

   public void setZ(double z)
   {
      position.setZ(z);
   }

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setPositionXY(Point2DReadOnly point)
   {
      position.setX(point.getX());
      position.setY(point.getY());
   }

   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      orientation.set(qx, qy, qy, qs);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      orientation.set(quaternion);
   }

   public void setOrientation(RotationMatrixReadOnly matrix3d)
   {
      orientation.set(matrix3d);
   }

   public void setOrientation(AxisAngleReadOnly axisAngle4d)
   {
      orientation.set(axisAngle4d);
   }

   public void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll);
   }

   public void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   @Override
   public void set(Pose other)
   {
      orientation.set(other.getOrientation());
      position.set(other.getPosition());
   }

   public void set(RigidBodyTransform transform)
   {
      transform.get(orientation, position);
   }

   public void set(QuaternionBasedTransform transform)
   {
      transform.get(orientation, position);
   }

   public void set(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   public void set(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /**
    * Normalizes the quaternion part of this pose to ensure it is a unit-quaternion describing a
    * proper orientation.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   public void normalizeQuaternion()
   {
      orientation.normalize();
   }

   /**
    * Normalizes the quaternion part of this pose and then limits the angle of the rotation it
    * represents to be &in; [-<i>pi</i>;<i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   public void normalizeQuaternionAndLimitToPi()
   {
      orientation.normalizeAndLimitToPiMinusPi();
   }

   public void interpolate(Pose other, double alpha)
   {
      position.interpolate(other.position, alpha);
      orientation.interpolate(other.orientation, alpha);
   }

   public void interpolate(Pose pose1, Pose pose2, double alpha)
   {
      position.interpolate(pose1.position, pose2.position, alpha);
      orientation.interpolate(pose1.orientation, pose2.orientation, alpha);
   }

   public void prependTranslation(double x, double y, double z)
   {
      position.add(x, y, z);
   }

   public void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   public void prependRotation(QuaternionReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   public void prependRotation(RotationMatrixReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   public void prependRotation(AxisAngleReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, position, position);
      orientation.prependYawRotation(yaw);
   }

   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, position, position);
      orientation.prependPitchRotation(pitch);
   }

   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, position, position);
      orientation.prependRollRotation(roll);
   }

   public void prependTransform(RigidBodyTransform transform)
   {
      applyTransform(transform);
   }

   public void prependTransform(QuaternionBasedTransform transform)
   {
      applyTransform(transform);
   }

   public void appendTranslation(double x, double y, double z)
   {
      double thisX = position.getX();
      double thisY = position.getY();
      double thisZ = position.getZ();

      position.set(x, y, z);
      orientation.transform(position);
      position.add(thisX, thisY, thisZ);
   }

   public void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   public void appendRotation(QuaternionReadOnly rotation)
   {
      orientation.multiply(rotation);
   }

   public void appendRotation(RotationMatrixReadOnly rotation)
   {
      orientation.multiply(rotation);
   }

   public void appendYawRotation(double yaw)
   {
      orientation.appendYawRotation(yaw);
   }

   public void appendPitchRotation(double pitch)
   {
      orientation.appendPitchRotation(pitch);
   }

   public void appendRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, position, position);
      orientation.appendRollRotation(roll);
   }

   public void appendTransform(RigidBodyTransform transform)
   {
      QuaternionTools.addTransform(orientation, transform.getTranslationVector(), position);
      orientation.multiply(transform.getRotationMatrix());
   }

   public void appendTransform(QuaternionBasedTransform transform)
   {
      QuaternionTools.addTransform(orientation, transform.getTranslationVector(), position);
      orientation.multiply(transform.getQuaternion());
   }

   /**
    * Computes the distance between the position of this pose 3D and the given {@code point}.
    * 
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    */
   public double getPositionDistance(Point3DReadOnly point)
   {
      return position.distance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    * 
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    */
   public double getPositionDistance(Pose other)
   {
      return position.distance(other.position);
   }

   /**
    * Computes the smallest angle representing the difference between the orientation part of this
    * pose 3D and the give {@code orientation}.
    * 
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this} and {@code orientation}.
    */
   public double getOrientationDistance(QuaternionReadOnly orientation)
   {
      return this.orientation.distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 2D and {@code other}.
    * 
    * @param other the other pose 2D used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this.orientation} and
    *         {@code other.orientation}.
    */
   public double getOrientationDistance(Pose other)
   {
      return orientation.distance(other.orientation);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(orientation);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(position);
      transform.inverseTransform(orientation);
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   public double getX()
   {
      return position.getX();
   }

   public double getY()
   {
      return position.getY();
   }

   public double getZ()
   {
      return position.getZ();
   }

   public double getYaw()
   {
      return orientation.getYaw();
   }

   public double getPitch()
   {
      return orientation.getPitch();
   }

   public double getRoll()
   {
      return orientation.getRoll();
   }

   public void getPosition(Tuple3DBasics tupleToPack)
   {
      tupleToPack.set(position);
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      matrixToPack.set(orientation);
   }

   public void getOrientation(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(orientation);
   }

   public void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.set(orientation);
   }

   public void getOrientationYawPitchRoll(double[] yawPitchRollToPack)
   {
      orientation.getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and packs the orientation described by the quaternion part of this pose as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   public void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      orientation.get(rotationVectorToPack);
   }

   public void get(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }

   public void get(QuaternionBasedTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }

   /**
    * Tests on a per component basis, if this pose 3D is exactly equal to {@code other}.
    *
    * @param other the other pose 3D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Pose other)
   {
      if (other == null)
         return false;
      else
         return position.equals(other.position) && orientation.equals(other.orientation);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Pose)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Pose) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(Pose other, double epsilon)
   {
      return epsilonEquals(other, epsilon, epsilon);
   }

   public boolean epsilonEquals(Pose other, double positionEpsilon, double orientationEpsilon)
   {
      return position.epsilonEquals(other.position, positionEpsilon) && orientation.epsilonEquals(other.orientation, orientationEpsilon);
   }

   public String printOutPosition()
   {
      return position.toString();
   }

   public String printOutOrientation()
   {
      return orientation.toString();
   }

   /**
    * Provides a {@code String} representation of this pose 3D as follows:<br>
    * Pose 3D: position = (x, y, z), orientation = (qx, qy, qz, qs)
    *
    * @return the {@code String} representing this pose 3D.
    */
   @Override
   public String toString()
   {
      return "Pose 3D: position = " + position + ", orientation = " + orientation;
   }
}
