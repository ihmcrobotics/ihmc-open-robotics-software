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
import us.ihmc.robotics.geometry.interfaces.PoseTransform;
import us.ihmc.robotics.geometry.transformables.Pose;

public abstract class Shape3d<S extends Shape3d<S>> implements GeometryObject<S>, PoseTransform
{
   private final Pose pose;
   
   public Shape3d()
   {
      pose = new Pose();
   }
   
   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the point is inside.
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
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level. If epsilonToGrowObject is positive, then 
    * the object will be checked as being larger. If negative, then the object will be shrunk.
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
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * If you wish to know the surface normal, then subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting orthogonal point).
    */
   public final void orthogonalProjection(Point3DBasics pointToCheckAndPack)
   {
      transformToLocal(pointToCheckAndPack);
      orthogonalProjectionShapeFrame(pointToCheckAndPack);
      transformToWorld(pointToCheckAndPack);
   }

   protected abstract void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack);

   /**
    * Returns true if inside the Shape3d. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    * But if they are not packed when outside, then they should be set to NaN. If they are set to NaN and you really do wish to see
    * where they would project to, then call orthogonalProjection.
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
      pose.applyTransform(transform);
   }

   public boolean epsilonEqualsPose(Shape3d<S> other, double epsilon)
   {
      return pose.epsilonEquals(other.pose, epsilon);
   }

   public void setPose(Shape3d<S> other)
   {
      pose.set(other.pose);
   }

   public boolean poseContainsNaN()
   {
      return pose.containsNaN();
   }

   public void setPoseToNaN()
   {
      pose.setToNaN();
   }

   public void setPoseToZero()
   {
      pose.setToZero();
   }
   
   // Pose API
   
   public void setX(double x)
   {
      pose.setX(x);
   }

   public void setY(double y)
   {
      pose.setY(y);
   }

   public void setZ(double z)
   {
      pose.setZ(z);
   }

   public double getX()
   {
      return pose.getX();
   }

   public double getY()
   {
      return pose.getY();
   }

   public double getZ()
   {
      return pose.getZ();
   }

   public void setPosition(double x, double y, double z)
   {
      pose.setPosition(x, y, z);
   }

   public void translate(Tuple3DReadOnly translation)
   {
      pose.translate(translation);
   }

   public void translate(double x, double y, double z)
   {
      pose.translate(x, y, z);
   }
   
   public Point3DReadOnly getPosition()
   {
      return pose.getPoint();
   }

   public QuaternionReadOnly getOrientation()
   {
      return pose.getOrientation();
   }
   
   public void applyTransformToPositionOnly(Transform transform)
   {
      pose.applyTransformToPositionOnly(transform);
   }

   public void applyTransformToOrientationOnly(Transform transform)
   {
      pose.applyTransformToOrientationOnly(transform);
   }
   
    public void appendTransform(RigidBodyTransform transform)
   {
      pose.appendTransform(transform);
   }

   public void appendTransform(QuaternionBasedTransform transform)
   {
      pose.appendTransform(transform);
   }
   
   public void interpolate(Shape3d<S> shape1, Shape3d<S> shape2, double alpha)
   {
      pose.interpolate(shape1.pose, shape2.pose, alpha);
   }
   
   public String getPositionString()
   {
      return pose.printOutPosition();
   }

   public String getOrientationString()
   {
      return pose.printOutOrientation();
   }
   
   public String getPoseString()
   {
      return pose.toString();
   }

   public void setPose(Pose other)
   {
      pose.set(other);
   }
   
   public void setPose(RigidBodyTransform transform)
   {
      pose.setPose(transform);
   }

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      pose.setPose(position, orientation);
   }

   public void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      pose.setPose(position, orientation);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      pose.setPosition(position);
   }

   public void setXY(Point2DReadOnly point)
   {
      pose.setXY(point);
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      pose.getPose(transformToPack);
   }

   public void getPosition(Tuple3DBasics tupleToPack)
   {
      pose.getPosition(tupleToPack);
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      pose.getRigidBodyTransform(transformToPack);
   }

   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      pose.setOrientation(qx, qy, qy, qs);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      pose.setOrientation(quaternion);
   }

   public void setOrientation(RotationMatrixReadOnly matrix3d)
   {
      pose.setOrientation(matrix3d);
   }

   public void setOrientation(AxisAngleReadOnly axisAngle4d)
   {
      pose.setOrientation(axisAngle4d);
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      pose.getOrientation(matrixToPack);
   }

   public void getOrientation(QuaternionBasics quaternionToPack)
   {
      pose.getOrientation(quaternionToPack);
   }

   public void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      pose.getOrientation(axisAngleToPack);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      pose.setYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      pose.setYawPitchRoll(yaw, pitch, roll);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      pose.getYawPitchRoll(yawPitchRollToPack);
   }

   public double getYaw()
   {
      return pose.getYaw();
   }

   public double getPitch()
   {
      return pose.getPitch();
   }

   public double getRoll()
   {
      return pose.getRoll();
   }
   
   // Pose transform

   @Override
   public void transformToWorld(Transformable transformable)
   {
      pose.transformToWorld(transformable);
   }

   @Override
   public void transformToWorld(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pose.transformToWorld(pointOriginal, pointTransformed);
   }

   @Override
   public void transformToWorld(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      pose.transformToWorld(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transformToWorld(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      pose.transformToWorld(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void transformToWorld(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      pose.transformToWorld(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pose.transformToWorld(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      pose.transformToWorld(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void transformToWorld(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      pose.transformToWorld(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transformToWorld(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      pose.transformToWorld(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transformToWorld(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      pose.transformToWorld(original, transformed);
   }

   @Override
   public void transformToWorld(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      pose.transformToWorld(original, transformed);
   }

   @Override
   public void transformToLocal(Transformable transformable)
   {
      pose.transformToLocal(transformable);
   }

   @Override
   public void transformToLocal(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pose.transformToLocal(pointOriginal, pointTransformed);
   }

   @Override
   public void transformToLocal(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      pose.transformToLocal(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transformToLocal(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      pose.transformToLocal(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void transformToLocal(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      pose.transformToLocal(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pose.transformToLocal(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      pose.transformToLocal(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void transformToLocal(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      pose.transformToLocal(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transformToLocal(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      pose.transformToLocal(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transformToLocal(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      pose.transformToLocal(original, transformed);
   }

   @Override
   public void transformToLocal(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      pose.transformToLocal(original, transformed);
   }
}
