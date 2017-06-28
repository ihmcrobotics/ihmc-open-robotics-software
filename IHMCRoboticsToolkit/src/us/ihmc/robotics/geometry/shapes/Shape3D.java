package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.TransformationTools.*;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Base implementation for 3D shapes such as: cylinder, box, sphere, etc.
 *
 * @param <S>
 */
public abstract class Shape3D<S extends Shape3D<S>> implements GeometryObject<S>
{
   protected final RigidBodyTransform shapePose = new RigidBodyTransform();

   public Shape3D()
   {
   }

   /**
    * Evaluates the query point {@code pointToCheck}:
    * <ul>
    * <li>tests if the query is located inside this shape,
    * <li>calculates the coordinates of the closest point to the query and on laying on the shape
    * surface,
    * <li>calculates the normal of the shape surface at the coordinates of the closest point to the
    * query.
    * </ul>
    *
    * @param pointToCheck the coordinates of the query to be evaluated. Not modified.
    * @param closestPointOnSurfaceToPack the closest point to the query that lies on the shape
    *           surface. Modified.
    * @param normalAtClosestPointToPack the surface normal at the closest point to the query. The
    *           normal points toward outside the shape. Modified.
    * @return {@code true} if the query is inside this shape or exactly on its surface,
    *         {@code false} otherwise.
    */
   public final boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      double xLocal = computeTransformedX(shapePose, true, pointToCheck);
      double yLocal = computeTransformedY(shapePose, true, pointToCheck);
      double zLocal = computeTransformedZ(shapePose, true, pointToCheck);

      boolean isInside = evaluateQuery(xLocal, yLocal, zLocal, closestPointOnSurfaceToPack, normalAtClosestPointToPack) <= 0.0;

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return shapePose.containsNaN();
   }

   /**
    * Calculates the minimum distance between the point and this shape.
    * <p>
    * Note that if the point is inside this shape, this method returns 0.0.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this shape.
    */
   public final double distance(Point3DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   /**
    * Returns minimum distance between the point and this shape.
    * <p>
    * The returned value is negative if the point is inside the shape.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the distance between the query and the shape, it is negative if the point is inside
    *         the shape.
    */
   public final double signedDistance(Point3DReadOnly point)
   {
      double xLocal = computeTransformedX(shapePose, true, point);
      double yLocal = computeTransformedY(shapePose, true, point);
      double zLocal = computeTransformedZ(shapePose, true, point);

      return evaluateQuery(xLocal, yLocal, zLocal, null, null);
   }

   /**
    * Tests separately and on a per component basis if the orientation and the position of this
    * shape's pose and {@code other}'s pose are equal to an {@code epsilon}.
    * 
    * @param other the other shape which its pose is to be compared against this shape's pose. Not
    *           modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two poses are equal component-wise, {@code false} otherwise.
    */
   public boolean epsilonEqualsPose(Shape3D<S> other, double epsilon)
   {
      return shapePose.epsilonEquals(other.shapePose, epsilon);
   }

   /**
    * Internal generic method used for the public API of any {@code Shape3d}.
    * 
    * @param x the x-coordinate of the query expressed in the local coordinates of this shape.
    * @param y the y-coordinate of the query expressed in the local coordinates of this shape.
    * @param z the z-coordinate of the query expressed in the local coordinates of this shape.
    * @param closestPointOnSurfaceToPack closest point to the query expressed in the local
    *           coordinates of this shape. Modified. Can be {@code null}.
    * @param normalAtClosestPointToPack normal of the shape surface at the closest point. Modified.
    *           Can be {@code null}.
    * @return the distance from the query to the closest point on the shape surface. The returned
    *         value is expected to be negative when the query is inside the shape.
    */
   protected abstract double evaluateQuery(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack);

   /**
    * Tests whether the given point is inside this shape or on its surface.
    *
    * @param query the coordinates of the query. Not modified.
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public final boolean isInsideOrOnSurface(Point3DReadOnly query)
   {
      return isInsideEpsilon(query, Epsilons.ONE_TRILLIONTH);
   }

   /**
    * Tests if the {@code query} is located inside this shape given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon > 0}, the size of this shape is increased by shifting its surface/faces
    * by a distance of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this shape is reduced by shifting its surface/faces by
    * a distance of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside this shape, {@code false}
    *         otherwise.
    */
   public final boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      double xLocal = computeTransformedX(shapePose, true, query);
      double yLocal = computeTransformedY(shapePose, true, query);
      double zLocal = computeTransformedZ(shapePose, true, query);

      return isInsideEpsilonShapeFrame(xLocal, yLocal, zLocal, epsilon);
   }

   /**
    * Tests if the {@code query} is located inside this shape given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon > 0}, the size of this shape is increased by shifting its surface/faces
    * by a distance of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this shape is reduced by shifting its surface/faces by
    * a distance of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    * 
    * @param x the x-coordinate of the query expressed in the local coordinates of this shape.
    * @param y the y-coordinate of the query expressed in the local coordinates of this shape.
    * @param z the z-coordinate of the query expressed in the local coordinates of this shape.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside this shape, {@code false}
    *         otherwise.
    */
   protected abstract boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon);

   /**
    * Computes the orthogonal projection of a point on this shape.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the query is inside the shape, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this shape. Modified.
    * @return whether the method succeeded or not.
    */
   public final boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a point this shape.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the query is inside the shape, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this shape is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    */
   public final boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      double xOriginal = pointToProject.getX();
      double yOriginal = pointToProject.getY();
      double zOriginal = pointToProject.getZ();

      double xLocal = computeTransformedX(shapePose, true, pointToProject);
      double yLocal = computeTransformedY(shapePose, true, pointToProject);
      double zLocal = computeTransformedZ(shapePose, true, pointToProject);

      boolean isInside = evaluateQuery(xLocal, yLocal, zLocal, projectionToPack, null) <= 0.0;

      if (isInside)
         projectionToPack.set(xOriginal, yOriginal, zOriginal);
      else
         transformToWorld(projectionToPack);

      return !isInside;
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * 
    * @param axisAngle the axis-angle holding the new orientation for this shape. Not modified.
    */
   public final void setOrientation(AxisAngleReadOnly axisAngle)
   {
      shapePose.setRotation(axisAngle);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * 
    * @param quaternion the quaternion holding the new orientation for this shape. Not modified.
    */
   public final void setOrientation(QuaternionReadOnly quaternion)
   {
      shapePose.setRotation(quaternion);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * 
    * @param rotationMatrix the rotation matrix holding the new orientation for this shape. Not
    *           modified.
    */
   public final void setOrientation(RotationMatrixReadOnly rotationMatrix)
   {
      shapePose.setRotation(rotationMatrix);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * <p>
    * The given {@code yaw}, {@code pitch}, and {@code roll} angles are composed into a rotation
    * matrix as follows:
    * 
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public final void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      shapePose.setRotationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * <p>
    * The given {@code yaw}, {@code pitch}, and {@code roll} angles are composed into a rotation
    * matrix as follows:
    * 
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    * 
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   public final void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      shapePose.setRotationYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    * 
    * @param pose pose holding the new position and orientation for this shape. Not modified.
    */
   public final void setPose(Pose3D pose)
   {
      pose.get(shapePose);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    * 
    * @param rigidBodyTransform rigid-body transform holding the new position and orientation for
    *           this shape. Not modified.
    */
   public final void setPose(RigidBodyTransform rigidBodyTransform)
   {
      shapePose.set(rigidBodyTransform);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape to the pose of {@code other}.
    * 
    * @param other shape holding the pose with the new position and orientation for this shape. Not
    *           modified.
    */
   public void setPose(Shape3D<S> other)
   {
      shapePose.set(other.shapePose);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    * 
    * @param position the new position for this shape. Not modified.
    * @param axisAngle the axis-angle holding the new orientation for this shape. Not modified.
    */
   public final void setPose(Tuple3DReadOnly position, AxisAngleReadOnly axisAngle)
   {
      shapePose.set(axisAngle, position);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    * 
    * @param position the new position for this shape. Not modified.
    * @param quaternion the quaternion holding the new orientation for this shape. Not modified.
    */
   public final void setPose(Tuple3DReadOnly position, QuaternionReadOnly quaternion)
   {
      shapePose.set(quaternion, position);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    * 
    * @param position the new position for this shape. Not modified.
    * @param rotationMatrix the rotation matrix holding the new orientation for this shape. Not
    *           modified.
    */
   public final void setPose(Tuple3DReadOnly position, RotationMatrixReadOnly rotationMatrix)
   {
      shapePose.set(rotationMatrix, position);
   }

   /**
    * Sets the position of this shape.
    * <p>
    * This method does not affect the orientation of this shape.
    * </p>
    * 
    * @param x the x-coordinate for this shape position.
    * @param y the y-coordinate for this shape position.
    * @param z the z-coordinate for this shape position.
    */
   public final void setPosition(double x, double y, double z)
   {
      shapePose.setTranslation(x, y, z);
   }

   /**
    * Sets the position of this shape.
    * <p>
    * This method does not affect the orientation of this shape.
    * </p>
    * 
    * @param position the new position for this shape. Not modified.
    */
   public final void setPosition(Tuple3DReadOnly position)
   {
      shapePose.setTranslation(position);
   }

   /**
    * Sets the x-coordinate of this shape position.
    * 
    * @param x the new x-coordinate for this shape.
    */
   public final void setPositionX(double x)
   {
      shapePose.setTranslationX(x);
   }

   /**
    * Sets the x and y coordinates of this shape position.
    * 
    * @param point2D point holding the new x and y coordinates for this shape position. Not
    *           modified.
    */
   public final void setPositionXY(Point2DReadOnly point2D)
   {
      shapePose.setTranslationX(point2D.getX());
      shapePose.setTranslationY(point2D.getY());
   }

   /**
    * Sets the y-coordinate of this shape position.
    * 
    * @param y the new y-coordinate for this shape.
    */
   public final void setPositionY(double y)
   {
      shapePose.setTranslationY(y);
   }

   /**
    * Sets the z-coordinate of this shape position.
    * 
    * @param z the new z-coordinate for this shape.
    */
   public final void setPositionZ(double z)
   {
      shapePose.setTranslationZ(z);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      shapePose.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      shapePose.setToZero();
   }

   /**
    * Gets the read-only reference to the orientation of this shape.
    * 
    * @return the orientation of this shape.
    */
   public RotationMatrixReadOnly getOrientation()
   {
      return shapePose.getRotationMatrix();
   }

   /**
    * Packs the orientation of this shape into an axis-angle.
    *
    * @param axisAngleToPack the axis-angle that is set to the orientation of this shape. Modified.
    */
   public final void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      shapePose.getRotation(axisAngleToPack);
   }

   /**
    * Packs the orientation of this shape into a quaternion.
    *
    * @param quaternionToPack the quaternion that is set to the orientation of this shape. Modified.
    */
   public final void getOrientation(QuaternionBasics quaternionToPack)
   {
      shapePose.getRotation(quaternionToPack);
   }

   /**
    * Packs the orientation of this shape into a rotation matrix.
    *
    * @param rotationMatrixToPack the matrix in which the orientation of this shape is stored.
    *           Modified.
    */
   public final void getOrientation(RotationMatrix matrixToPack)
   {
      shapePose.getRotation(matrixToPack);
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   public final double getOrientationPitch()
   {
      return shapePose.getRotationMatrix().getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   public final double getOrientationRoll()
   {
      return shapePose.getRotationMatrix().getRoll();
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   public final double getOrientationYaw()
   {
      return shapePose.getRotationMatrix().getYaw();
   }

   /**
    * Computes and packs the orientation of this shape into the given yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   public final void getOrientationYawPitchRoll(double[] yawPitchRollToPack)
   {
      shapePose.getRotationYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Packs the pose of this shape into the given {@code poseToPack}.
    * 
    * @param poseToPack the pose in which the position and orientation of this shape are stored.
    *           Modified.
    */
   public final void getPose(Pose3D poseToPack)
   {
      poseToPack.set(shapePose);
   }

   /**
    * Packs the pose of this shape into the given {@code transformToPack}.
    * 
    * @param transformToPack the rigid-body transform in which the position and orientation of this
    *           shape are stored. Modified.
    */
   public final void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(shapePose);
   }

   /**
    * Provides a {@code String} representation of this shape pose as follows: <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this shape pose.
    */
   public String getPoseString()
   {
      return shapePose.toString();
   }

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   public Tuple3DReadOnly getPosition()
   {
      return shapePose.getTranslationVector();
   }

   /**
    * Packs the position of this shape.
    *
    * @param tupleToPack the tuple in which the position of this shape is stored. Modified.
    */
   public final void getPosition(Tuple3DBasics tupleToPack)
   {
      shapePose.getTranslation(tupleToPack);
   }

   /**
    * Gets the x-coordinate of the position of this shape.
    * 
    * @return the x-coordinate of this shape position.
    */
   public final double getPositionX()
   {
      return shapePose.getTranslationX();
   }

   /**
    * Gets the y-coordinate of the position of this shape.
    * 
    * @return the y-coordinate of this shape position.
    */
   public final double getPositionY()
   {
      return shapePose.getTranslationY();
   }

   /**
    * Gets the z-coordinate of the position of this shape.
    * 
    * @return the z-coordinate of this shape position.
    */
   public final double getPositionZ()
   {
      return shapePose.getTranslationZ();
   }

   /**
    * Transforms this shape with a quaternion based transform that is defined in the local
    * coordinates of this shape.
    * 
    * @param transform the transform to append to this shape pose. Not modified.
    */
   public final void appendTransform(QuaternionBasedTransform transform)
   {
      shapePose.multiply(transform);
   }

   /**
    * Transforms this shape with a rigid-body transform that is defined in the local coordinates of
    * this shape.
    * 
    * @param transform the transform to append to this shape pose. Not modified.
    */
   public final void appendTransform(RigidBodyTransform transform)
   {
      shapePose.multiply(transform);
   }

   /**
    * Translate this shape with the given (x, y, z) components that are expressed in the local
    * coordinates of this shape.
    * 
    * @param x the x-component of the translation to append to this shape pose.
    * @param y the y-component of the translation to append to this shape pose.
    * @param z the z-component of the translation to append to this shape pose.
    */
   public final void appendTranslation(double x, double y, double z)
   {
      shapePose.appendTranslation(x, y, z);
   }

   /**
    * Translate this shape with the given {@code translation} that is expressed in the local
    * coordinates of this shape.
    *
    * @param translation the translation to append to the pose of this shape. Not modified.
    */
   public final void appendTranslation(Tuple3DReadOnly translation)
   {
      shapePose.appendTranslation(translation);
   }

   /**
    * Rotates this shape by angle of {@code yaw} about the z-axis of this shape local coordinates.
    * 
    * @param yaw the angle to rotate about the local z-axis.
    */
   public final void appendYawRotation(double yaw)
   {
      shapePose.appendYawRotation(yaw);
   }

   /**
    * Rotates this shape by angle of {@code pitch} about the y-axis of this shape local coordinates.
    * 
    * @param pitch the angle to rotate about the local y-axis.
    */
   public final void appendPitchRotation(double pitch)
   {
      shapePose.appendPitchRotation(pitch);
   }

   /**
    * Rotates this shape by angle of {@code roll} about the x-axis of this shape local coordinates.
    * 
    * @param roll the angle to rotate about the local x-axis.
    */
   public final void appendRollRotation(double roll)
   {
      shapePose.appendRollRotation(roll);
   }

   /**
    * Translates this shape with the given (x, y, z) components that are expressed in the world
    * coordinates.
    * 
    * @param x the x-component of the translation to prepend to this shape pose.
    * @param y the y-component of the translation to prepend to this shape pose.
    * @param z the z-component of the translation to prepend to this shape pose.
    */
   public final void prependTranslation(double x, double y, double z)
   {
      shapePose.prependTranslation(x, y, z);
   }

   /**
    * Translate this shape with the given {@code translation} that is expressed in the world
    * coordinates.
    *
    * @param translation the translation to prepend to the pose of this shape. Not modified.
    */
   public final void prependTranslation(Tuple3DReadOnly translation)
   {
      shapePose.prependTranslation(translation);
   }

   /**
    * Rotates this shape by angle of {@code yaw} about the z-axis of the world coordinates.
    * 
    * @param yaw the angle to rotate about the local z-axis.
    */
   public final void prependYawRotation(double yaw)
   {
      shapePose.prependYawRotation(yaw);
   }

   /**
    * Rotates this shape by angle of {@code pitch} about the y-axis of the world coordinates.
    * 
    * @param pitch the angle to rotate about the local y-axis.
    */
   public final void prependPitchRotation(double pitch)
   {
      shapePose.prependPitchRotation(pitch);
   }

   /**
    * Rotates this shape by angle of {@code roll} about the x-axis of the world coordinates.
    * 
    * @param roll the angle to rotate about the local x-axis.
    */
   public final void prependRollRotation(double roll)
   {
      shapePose.prependRollRotation(roll);
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in
    * this shape local coordinates.
    * 
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   public final void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(shapePose);
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates
    * to being expressed in world.
    * 
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   public final void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(shapePose);
   }

   /** {@inheritDoc} */
   @Override
   public final void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(shapePose);
   }

   /** {@inheritDoc} */
   @Override
   public final void applyTransform(Transform transform)
   {
      transform.transform(shapePose);
   }
}
