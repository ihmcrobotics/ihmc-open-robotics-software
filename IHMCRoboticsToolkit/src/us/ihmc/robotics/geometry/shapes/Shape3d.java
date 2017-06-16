package us.ihmc.robotics.geometry.shapes;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.robotics.geometry.transformables.Pose;

public abstract class Shape3d<S extends Shape3d<S>> implements GeometryObject<S>
{
   private final QuaternionBasedTransform shapePose;

   public Shape3d()
   {
      shapePose = new QuaternionBasedTransform();
   }

   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the
    * point is inside.
    *
    * @param point
    * @return distance from the point to this Shape3d.
    */
   public final double distance(Point3DBasics point)
   {
      transformToLocal(point);
      double distance = distanceShapeFrame(point);
      transformToWorld(point);
      return distance;
   }

   protected abstract double distanceShapeFrame(Point3DReadOnly point);

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    *
    * @param pointToCheck
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public final boolean isInsideOrOnSurface(Point3DBasics pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, Epsilons.ONE_TRILLIONTH);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape, within a given
    * tolerance or error level. If epsilonToGrowObject is positive, then the object will be checked
    * as being larger. If negative, then the object will be shrunk.
    *
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(Point3DBasics pointToCheck, double epsilonToGrowObject)
   {
      transformToLocal(pointToCheck);
      boolean isInsideOrOnSurface = isInsideOrOnSurfaceShapeFrame(pointToCheck, epsilonToGrowObject);
      transformToWorld(pointToCheck);
      return isInsideOrOnSurface;
   }

   protected abstract boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilonToGrowObject);

   /**
    * Find the closest point on the surface of this shape to the given point. If the given point is
    * on or inside the shape, then it is not changed. If you wish to know the surface normal, then
    * subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check), and an output
    *           parameter (packed with the resulting orthogonal point).
    */
   public final void orthogonalProjection(Point3DBasics pointToCheckAndPack)
   {
      transformToLocal(pointToCheckAndPack);
      orthogonalProjectionShapeFrame(pointToCheckAndPack);
      transformToWorld(pointToCheckAndPack);
   }

   protected abstract void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack);

   /**
    * Returns true if inside the Shape3d. If inside, must pack the intersection and normal. If not
    * inside, packing those is optional. But if they are not packed when outside, then they should
    * be set to NaN. If they are set to NaN and you really do wish to see where they would project
    * to, then call orthogonalProjection.
    *
    * @param pointToCheck
    * @param intersectionToPack
    * @param normalToPack
    * @return true if the point is inside, false otherwise.
    */
   public final boolean checkIfInside(Point3DBasics pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      transformToLocal(pointToCheck);
      boolean isInside = checkIfInsideShapeFrame(pointToCheck, closestPointOnSurfaceToPack, normalToPack);
      //TODO: This modifies pointToCheck and transforms back. Should we make a temp variable instead, or are we trying to be Thread safe here?
      transformToWorld(pointToCheck);
      if (closestPointOnSurfaceToPack != null)
      {
         transformToWorld(closestPointOnSurfaceToPack);
      }
      if (normalToPack != null)
      {
         transformToWorld(normalToPack);
      }
      return isInside;
   }

   protected abstract boolean checkIfInsideShapeFrame(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack);

   public void applyTransformToPose(Transform transform)
   {
      transform.transform(shapePose);
   }

   public void applyInverseTransformToPose(Transform transform)
   {
      transform.inverseTransform(shapePose);
   }

   public boolean epsilonEqualsPose(Shape3d<S> other, double epsilon)
   {
      return shapePose.epsilonEquals(other.shapePose, epsilon);
   }

   public void setPose(Shape3d<S> other)
   {
      shapePose.set(other.shapePose);
   }

   public boolean poseContainsNaN()
   {
      return shapePose.containsNaN();
   }

   public void setPoseToNaN()
   {
      shapePose.setToNaN();
   }

   public void setPoseToZero()
   {
      shapePose.setToZero();
   }

   // Pose API

   public void setX(double x)
   {
      shapePose.setTranslationX(x);
   }

   public void setY(double y)
   {
      shapePose.setTranslationY(y);
   }

   public void setZ(double z)
   {
      shapePose.setTranslationZ(z);
   }

   public double getX()
   {
      return shapePose.getTranslationX();
   }

   public double getY()
   {
      return shapePose.getTranslationY();
   }

   public double getZ()
   {
      return shapePose.getTranslationZ();
   }

   public void setPosition(double x, double y, double z)
   {
      shapePose.setTranslation(x, y, z);
   }

   public void translate(Tuple3DReadOnly translation)
   {
      shapePose.prependTranslation(translation);
   }

   public void translate(double x, double y, double z)
   {
      shapePose.prependTranslation(x, y, z);
   }

   public void appendTransform(RigidBodyTransform transform)
   {
      shapePose.multiply(transform);
   }

   public void appendTransform(QuaternionBasedTransform transform)
   {
      shapePose.multiply(transform);
   }

   public void interpolate(Shape3d<S> shape1, Shape3d<S> shape2, double alpha)
   {
      shapePose.interpolate(shape1.shapePose, shape2.shapePose, alpha);
   }

   public String getPositionString()
   {
      return shapePose.getTranslationVector().toString();
   }

   public String getOrientationString()
   {
      return shapePose.getQuaternion().toString();
   }

   public String getPoseString()
   {
      return shapePose.toString();
   }

   public void setPose(Pose pose)
   {
      pose.getPose(this.shapePose);
   }

   public void setPose(RigidBodyTransform transform)
   {
      shapePose.set(transform);
   }

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      shapePose.set(orientation, position);
   }

   public void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      shapePose.set(orientation, position);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      shapePose.setTranslation(position);
   }

   public void setXY(Point2DReadOnly point)
   {
      shapePose.setTranslationX(point.getX());
      shapePose.setTranslationY(point.getY());
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(shapePose);
   }

   public void getPose(Pose poseToPack)
   {
      poseToPack.set(shapePose);
   }

   public void getPosition(Tuple3DBasics tupleToPack)
   {
      shapePose.getTranslation(tupleToPack);
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(shapePose);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      shapePose.setRotation(quaternion);
   }

   public void setOrientation(RotationMatrixReadOnly matrix3d)
   {
      shapePose.setRotation(matrix3d);
   }

   public void setOrientation(AxisAngleReadOnly axisAngle4d)
   {
      shapePose.setRotation(axisAngle4d);
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      shapePose.getRotation(matrixToPack);
   }

   public void getOrientation(QuaternionBasics quaternionToPack)
   {
      shapePose.getRotation(quaternionToPack);
   }

   public void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      shapePose.getRotation(axisAngleToPack);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      shapePose.setRotationYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      shapePose.setRotationYawPitchRoll(yaw, pitch, roll);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      shapePose.getRotationYawPitchRoll(yawPitchRollToPack);
   }

   public double getYaw()
   {
      return shapePose.getQuaternion().getYaw();
   }

   public double getPitch()
   {
      return shapePose.getQuaternion().getPitch();
   }

   public double getRoll()
   {
      return shapePose.getQuaternion().getRoll();
   }

   // Pose transform

   public void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(shapePose);
   }

   public void transformToWorld(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      shapePose.transform(pointOriginal, pointTransformed);
   }

   public void transformToWorld(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      shapePose.transform(vectorOriginal, vectorTransformed);
   }

   public void transformToWorld(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      shapePose.transform(quaternionOriginal, quaternionTransformed);
   }

   public void transformToWorld(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      shapePose.transform(vectorOriginal, vectorTransformed);
   }

   public void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   public void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   public void transformToWorld(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      shapePose.transform(matrixOriginal, matrixTransformed);
   }

   public void transformToWorld(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      shapePose.transform(matrixOriginal, matrixTransformed);
   }

   public void transformToWorld(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      shapePose.transform(original, transformed);
   }

   public void transformToWorld(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      shapePose.transform(original, transformed);
   }

   public void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(shapePose);
   }

   public void transformToLocal(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      shapePose.inverseTransform(pointOriginal, pointTransformed);
   }

   public void transformToLocal(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public void transformToLocal(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      shapePose.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   public void transformToLocal(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   public void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   public void transformToLocal(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      shapePose.inverseTransform(matrixOriginal, matrixTransformed);
   }

   public void transformToLocal(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      shapePose.inverseTransform(matrixOriginal, matrixTransformed);
   }

   public void transformToLocal(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      shapePose.inverseTransform(original, transformed);
   }

   public void transformToLocal(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      shapePose.inverseTransform(original, transformed);
   }
}
