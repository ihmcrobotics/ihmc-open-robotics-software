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
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * A {@code Pose3D} represents a position and orientation in 3 dimensions.
 */
public class Pose3D implements GeometryObject<Pose3D>
{
   /** The position part of this pose 3D. */
   private final Point3D position = new Point3D();
   /** The orientation part of this pose 3D. */
   private final Quaternion orientation = new Quaternion();

   /**
    * Creates a new pose 3D initialized with its position at (0, 0, 0) and orientation set to the
    * neutral quaternion, i.e. zero rotation.
    */
   public Pose3D()
   {
   }

   /**
    * Creates a new pose 3D and initializes it with the given parameters.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * 
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    * @param yaw the first angle from the yaw-pitch-roll representation, represents a rotation about
    *           the z-axis.
    * @param pitch the second angle from the yaw-pitch-roll representation, represents a rotation
    *           about the y-axis.
    * @param roll the third angle from the yaw-pitch-roll representation, represents a rotation
    *           about the x-axis.
    */
   public Pose3D(double x, double y, double z, double yaw, double pitch, double roll)
   {
      set(x, y, z, yaw, pitch, roll);
   }

   /**
    * Creates a new pose 3D and initializes it to {@code other}.
    * 
    * @param other the other pose 3D used to initialize this. Not modified.
    */
   public Pose3D(Pose3D other)
   {
      set(other);
   }

   /**
    * Creates a new pose 3D and initializes it with the given rigid-body transform.
    * 
    * @param rigidBodyTransform the transform used to initialize this. Not modified.
    */
   public Pose3D(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new pose 3D and initializes it with the given parameters.
    * 
    * @param position tuple used to initialize the position part of this pose. Not modified.
    * @param orientation used to initialize the orientation part of this pose. Not modified.
    */
   public Pose3D(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return orientation.containsNaN() || position.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      orientation.setToNaN();
      position.setToNaN();
   }

   /**
    * Sets the position to (0, 0) and the orientation to the neutral quaternion, i.e. zero rotation.
    */
   @Override
   public void setToZero()
   {
      orientation.setToZero();
      position.setToZero();
   }

   /**
    * Sets the x-coordinate of the position.
    * 
    * @param x the x-coordinate of the position.
    */
   public void setX(double x)
   {
      position.setX(x);
   }

   /**
    * Sets the y-coordinate of the position.
    * 
    * @param y the y-coordinate of the position.
    */
   public void setY(double y)
   {
      position.setY(y);
   }

   /**
    * Sets the z-coordinate of the position.
    * 
    * @param z the z-coordinate of the position.
    */
   public void setZ(double z)
   {
      position.setZ(z);
   }

   /**
    * Sets the position coordinates.
    * 
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    */
   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   /**
    * Sets the position to the given tuple.
    * 
    * @param position the tuple with the new position coordinates. Not modified.
    */
   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   /**
    * Sets the x and y coordinates from the given tuple 2D, the z coordinate remains unchanged.
    * 
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    */
   public void setPositionXY(Tuple2DReadOnly position2D)
   {
      position.setX(position2D.getX());
      position.setY(position2D.getY());
   }

   /**
    * Sets the orientation part of this pose 3D with the 4 components of a quaternion.
    * <p>
    * The quaternion is normalized internally.
    * </p>
    * 
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    */
   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      orientation.set(qx, qy, qy, qs);
   }

   /**
    * Sets the orientation part of this pose 3D with the given quaternion.
    * 
    * @param orientation the quaternion used to set this pose's orientation. Not modified.
    */
   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given rotation matrix.
    * 
    * @param orientation the rotation matrix used to set this pose's orientation. Not modified.
    */
   public void setOrientation(RotationMatrixReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given axis-angle.
    * 
    * @param orientation the axis-angle used to set this pose's orientation. Not modified.
    */
   public void setOrientation(AxisAngleReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * 
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   public void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets all the components of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * 
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void set(double x, double y, double z, double yaw, double pitch, double roll)
   {
      setPosition(x, y, z);
      setOrientationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    * 
    * @param other the other pose 3D. Not modified.
    */
   @Override
   public void set(Pose3D other)
   {
      orientation.set(other.getOrientation());
      position.set(other.getPosition());
   }

   /**
    * Sets this pose 3D to match the given rigid-body transform.
    * 
    * @param rigidBodyTransform the transform use to set this pose 3D. Not modified.
    */
   public void set(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.get(orientation, position);
   }

   /**
    * Sets this pose 3D to match the given quaternion-based transform.
    * 
    * @param quaternionBasedTransform the transform use to set this pose 3D. Not modified.
    */
   public void set(QuaternionBasedTransform quaternionBasedTransform)
   {
      quaternionBasedTransform.get(orientation, position);
   }

   /**
    * Sets both position and orientation.
    * 
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    */
   public void set(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /**
    * Sets both position and orientation.
    * 
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the rotation matrix with the new orientation. Not modified.
    */
   public void set(Tuple3DReadOnly position, RotationMatrixReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /**
    * Sets both position and orientation.
    * 
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the axis-angle with the new orientation. Not modified.
    */
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
      orientation.normalizeAndLimitToPi();
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    * 
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   public void interpolate(Pose3D other, double alpha)
   {
      position.interpolate(other.position, alpha);
      orientation.interpolate(other.orientation, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    * 
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    */
   public void interpolate(Pose3D pose1, Pose3D pose2, double alpha)
   {
      position.interpolate(pose1.position, pose2.position, alpha);
      orientation.interpolate(pose1.orientation, pose2.orientation, alpha);
   }

   /**
    * Adds the translation (x, y, z) to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the translation is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(double, double, double)}.
    * </p>
    * 
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   public void prependTranslation(double x, double y, double z)
   {
      position.add(x, y, z);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates
    * in which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D,
    * use {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    * 
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   public void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    * 
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   public void prependRotation(QuaternionReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    * 
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   public void prependRotation(RotationMatrixReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    * 
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   public void prependRotation(AxisAngleReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(orientation);
   }

   /**
    * Prepends a rotation about the z-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    * 
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, position, position);
      orientation.prependYawRotation(yaw);
   }

   /**
    * Prepends a rotation about the y-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    * 
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, position, position);
      orientation.prependPitchRotation(pitch);
   }

   /**
    * Prepends a rotation about the x-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    * 
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, position, position);
      orientation.prependRollRotation(roll);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    * 
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   public void prependTransform(RigidBodyTransform transform)
   {
      applyTransform(transform);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    * 
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   public void prependTransform(QuaternionBasedTransform transform)
   {
      applyTransform(transform);
   }

   /**
    * Rotates, then adds the translation (x, y, z) to this pose 3D.
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described
    * by this pose 3D. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    * 
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   public void appendTranslation(double x, double y, double z)
   {
      double thisX = position.getX();
      double thisY = position.getY();
      double thisZ = position.getZ();

      position.set(x, y, z);
      orientation.transform(position);
      position.add(thisX, thisY, thisZ);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(Tuple2DReadOnly)}.
    * </p>
    * 
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   public void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    * 
    * @param rotation the rotation to append to this pose 3D. Not modified.
    */
   public void appendRotation(QuaternionReadOnly rotation)
   {
      orientation.multiply(rotation);
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    * 
    * @param rotation the rotation to append to this pose 3D. Not modified.
    */
   public void appendRotation(RotationMatrixReadOnly rotation)
   {
      orientation.multiply(rotation);
   }

   /**
    * Appends a rotation about the z-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * 
    * <pre>
    *                                       / cos(yaw) -sin(yaw) 0 \
    * this.orientation = this.orientation * | sin(yaw)  cos(yaw) 0 |
    *                                       \    0         0     1 /
    * </pre>
    * </p>
    * 
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      orientation.appendYawRotation(yaw);
   }

   /**
    * Appends a rotation about the y-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * 
    * <pre>
    *                                       /  cos(pitch) 0 sin(pitch) \
    * this.orientation = this.orientation * |      0      1     0      |
    *                                       \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * </p>
    * 
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      orientation.appendPitchRotation(pitch);
   }

   /**
    * Appends a rotation about the x-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * 
    * <pre>
    *                                       / 1     0          0     \
    * this.orientation = this.orientation * | 0 cos(roll) -sin(roll) |
    *                                       \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    * 
    * @param roll the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, position, position);
      orientation.appendRollRotation(roll);
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    * 
    * @param transform the rigid-body transform to append to this pose 3D. Not modified.
    */
   public void appendTransform(RigidBodyTransform transform)
   {
      QuaternionTools.addTransform(orientation, transform.getTranslationVector(), position);
      orientation.multiply(transform.getRotationMatrix());
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    * 
    * @param transform the quaternion-based transform to append to this pose 3D. Not modified.
    */
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
   public double getPositionDistance(Pose3D other)
   {
      return position.distance(other.position);
   }

   /**
    * Computes the smallest angle representing the difference between the orientation part of this
    * pose 3D and the give {@code orientation}.
    * 
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this} and {@code orientation}, it is contained in
    *         [0, <i>pi</i>].
    */
   public double getOrientationDistance(QuaternionReadOnly orientation)
   {
      return this.orientation.distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 3D and {@code other}.
    * 
    * @param other the other pose 3D used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this.orientation} and {@code other.orientation},
    *         it is contained in [0, <i>pi</i>].
    */
   public double getOrientationDistance(Pose3D other)
   {
      return orientation.distance(other.orientation);
   }

   /**
    * Gets the read-only reference of the position part of this pose 3D.
    *
    * @return the position part of this pose 3D.
    */
   public Point3DReadOnly getPosition()
   {
      return position;
   }

   /**
    * Gets the read-only reference to the orientation part of this pose 3D.
    *
    * @return the orientation part of this pose 3D.
    */
   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   /**
    * Gets the x-coordinate of the position part of this pose 3D.
    * 
    * @return the x-coordinate of this pose 3D.
    */
   public double getX()
   {
      return position.getX();
   }

   /**
    * Gets the y-coordinate of the position part of this pose 3D.
    * 
    * @return the y-coordinate of this pose 3D.
    */
   public double getY()
   {
      return position.getY();
   }

   /**
    * Gets the z-coordinate of the position part of this pose 3D.
    * 
    * @return the z-coordinate of this pose 3D.
    */
   public double getZ()
   {
      return position.getZ();
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   public double getYaw()
   {
      return orientation.getYaw();
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   public double getPitch()
   {
      return orientation.getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   public double getRoll()
   {
      return orientation.getRoll();
   }

   /**
    * Packs the position part of this pose 3D into the given {@code positionToPack}.
    * 
    * @param positionToPack tuple used to store the position coordinates. Modified.
    */
   public void getPosition(Tuple3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    * 
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   public void getOrientation(RotationMatrix orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    * 
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    * 
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   public void getOrientation(AxisAngleBasics orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose 3D as the
    * yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   public void getOrientationYawPitchRoll(double[] yawPitchRollToPack)
   {
      orientation.getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose as a
    * rotation vector.
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

   /**
    * Packs this pose 3D into the given {@code transformToPack}.
    * 
    * @param transformToPack the rigid-body transform that is set to represent this pose 3D.
    *           Modified.
    */
   public void get(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }

   /**
    * Packs this pose 3D into the given {@code transformToPack}.
    * 
    * @param transformToPack the quaternion-based transform that is set to represent this pose 3D.
    *           Modified.
    */
   public void get(QuaternionBasedTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the given {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(orientation);
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the inverse of the given
    * {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(position);
      transform.inverseTransform(orientation);
   }

   /**
    * Tests on a per component basis, if this pose 3D is exactly equal to {@code other}.
    *
    * @param other the other pose 3D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Pose3D other)
   {
      if (other == null)
         return false;
      else
         return position.equals(other.position) && orientation.equals(other.orientation);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Pose3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Pose3D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Pose3D other, double epsilon)
   {
      return epsilonEquals(other, epsilon, epsilon);
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    * 
    * @param other the query. Not modified.
    * @param positionEpsilon the tolerance to use for comparing the position part.
    * @param orientationEpsilon the tolerance to use for comparing the orientation part.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   public boolean epsilonEquals(Pose3D other, double positionEpsilon, double orientationEpsilon)
   {
      return position.epsilonEquals(other.position, positionEpsilon) && orientation.epsilonEquals(other.orientation, orientationEpsilon);
   }

   /**
    * Provides a {@code String} representation of the position part of this pose 3D as follows:<br>
    * (x, y, z)
    * 
    * @return the {@code String} representing the position part of this pose 3D.
    */
   public String printOutPosition()
   {
      return position.toString();
   }

   /**
    * Provides a {@code String} representation of the orientation part of this pose 3D as
    * follows:<br>
    * (qx, qy, qz, qs)
    * 
    * @return the {@code String} representing the orientation part of this pose 3D.
    */
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
