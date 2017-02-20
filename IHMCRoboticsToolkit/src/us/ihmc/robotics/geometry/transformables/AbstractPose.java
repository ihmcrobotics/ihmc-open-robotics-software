package us.ihmc.robotics.geometry.transformables;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.robotics.geometry.interfaces.PoseTransform;

public abstract class AbstractPose implements PoseTransform
{
   private final Quaternion orientation;
   private final Point3D position;
   private final PoseTransformMapping poseTransformMapping;

   public AbstractPose(Pose pose)
   {
      orientation = new Quaternion(pose.getOrientation());
      position = new Point3D(pose.getPosition());
      poseTransformMapping = new PoseTransformMapping();
   }

   public AbstractPose()
   {
      orientation = new Quaternion();
      position = new Point3D();
      poseTransformMapping = new PoseTransformMapping();
   }

   public AbstractPose(RigidBodyTransform transform)
   {
      transform.getRotation(orientation = new Quaternion());
      transform.getTranslation(position = new Point3D());
      poseTransformMapping = new PoseTransformMapping();
   }

   public AbstractPose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation = new Quaternion(orientation);
      this.position = new Point3D(position);
      poseTransformMapping = new PoseTransformMapping();
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

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void translate(Tuple3DReadOnly translation)
   {
      position.add(translation);
   }

   public void translate(double x, double y, double z)
   {
      position.add(x, y, z);
   }

   public Point3DReadOnly getPoint()
   {
      return position;
   }

   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   public void setPose(AbstractPose other)
   {
      orientation.set(other.getOrientation());
      position.set(other.getPosition());
   }

   public void setPoseToZero()
   {
      orientation.setToZero();
      position.setToZero();
   }

   public void setPoseToNaN()
   {
      orientation.setToNaN();
      position.setToNaN();
   }

   public boolean poseContainsNaN()
   {
      return orientation.containsNaN() || position.containsNaN();
   }

   public boolean epsilonEqualsPose(AbstractPose other, double epsilon)
   {
      return orientation.epsilonEquals(other.getOrientation(), epsilon) && position.epsilonEquals(other.getPosition(), epsilon);
   }

   public void applyTransformToPose(Transform transform)
   {
      transform.transform(position);
      transform.transform(orientation);
   }
   
   public void applyTransformToPositionOnly(Transform transform)
   {
      transform.transform(position);
   }

   public void applyTransformToOrientationOnly(Transform transform)
   {
      transform.transform(orientation);
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
   
   public void interpolate(AbstractPose pose1, AbstractPose pose2, double alpha)
   {
      orientation.interpolate(pose1.getOrientation(), pose2.getOrientation(), alpha);
      position.interpolate(pose1.getPosition(), pose2.getPosition(), alpha);
   }

   public String printOutPosition()
   {
      return position.toString();
   }

   public String printOutOrientation()
   {
      return orientation.toString();
   }

   public void setPose(RigidBodyTransform transform)
   {
      transform.getRotation(orientation);
      transform.getTranslation(position);
   }

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   public void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setXY(Point2DReadOnly point)
   {
      position.setX(point.getX());
      position.setY(point.getY());
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public void getPosition(Tuple3DBasics tupleToPack)
   {
      tupleToPack.set(position);
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
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

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      orientation.getYawPitchRoll(yawPitchRollToPack);
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

   public boolean epsilonEquals(AbstractPose other, double positionErrorMargin, double orientationErrorMargin)
   {
      return position.epsilonEquals(other.getPosition(), positionErrorMargin) && orientation.epsilonEquals(other.getOrientation(), orientationErrorMargin);
   }
   
   public String getPoseString()
   {
      return EuclidCoreIOTools.getTuple3DString(position) + "\n" + EuclidCoreIOTools.getTuple4DString(orientation);
   }
   
   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(poseTransformMapping);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      orientation.transform(pointOriginal, pointTransformed);
      pointTransformed.add(position);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      orientation.transform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      orientation.transform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      orientation.transform(vectorOriginal, vectorTransformed);
      vectorTransformed.addX(vectorTransformed.getS() * position.getX());
      vectorTransformed.addY(vectorTransformed.getS() * position.getY());
      vectorTransformed.addZ(vectorTransformed.getS() * position.getZ());
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      orientation.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      pointTransformed.add(position.getX(), position.getY());
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      orientation.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      orientation.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      orientation.transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(orientation, position);
      transformed.multiply(original);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToWorld(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(orientation, position);
      transformed.multiply(original);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Transformable transformable)
   {
      transformable.applyTransform(poseTransformMapping);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(position);
      orientation.inverseTransform(pointTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      orientation.inverseTransform(vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      orientation.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.set(vectorOriginal);
      vectorTransformed.subX(vectorTransformed.getS() * position.getX());
      vectorTransformed.subY(vectorTransformed.getS() * position.getY());
      vectorTransformed.subZ(vectorTransformed.getS() * position.getZ());
      orientation.inverseTransform(vectorTransformed, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(position.getX(), position.getY());
      orientation.inverseTransform(pointTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      orientation.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      orientation.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      orientation.inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      transformed.set(orientation, position);
      transformed.invert();
      transformed.multiply(original);
   }

   /** {@inheritDoc} */
   @Override
   public void transformToLocal(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      transformed.set(orientation, position);
      transformed.invert();
      transformed.multiply(original);
   }
   
   private class PoseTransformMapping implements Transform
   {
      @Override
      public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
      {
         transformToWorld(pointOriginal, pointTransformed);
      }

      @Override
      public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
      {
         transformToWorld(vectorOriginal, vectorTransformed);
      }

      @Override
      public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
      {
         transformToWorld(quaternionOriginal, quaternionTransformed);
      }

      @Override
      public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         transformToWorld(vectorOriginal, vectorTransformed);
      }

      @Override
      public void transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
      {
         transformToWorld(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      }

      @Override
      public void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
      {
         transformToWorld(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
      }

      @Override
      public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
      {
         transformToWorld(matrixOriginal, matrixTransformed);
      }

      @Override
      public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
      {
         transformToWorld(matrixOriginal, matrixTransformed);
      }

      @Override
      public void transform(RigidBodyTransform original, RigidBodyTransform transformed)
      {
         transformToWorld(original, transformed);
      }

      @Override
      public void transform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
      {
         transformToWorld(original, transformed);
      }

      @Override
      public void transform(AffineTransform original, AffineTransform transformed)
      {
         throw new RuntimeException("Affine transforms can't be transformed to Pose frame.");
      }

      @Override
      public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
      {
         transformToLocal(pointOriginal, pointTransformed);
      }

      @Override
      public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
      {
         transformToLocal(vectorOriginal, vectorTransformed);
      }

      @Override
      public void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
      {
         transformToLocal(quaternionOriginal, quaternionTransformed);
      }

      @Override
      public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
      {
         transformToLocal(vectorOriginal, vectorTransformed);
      }

      @Override
      public void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
      {
         transformToLocal(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      }

      @Override
      public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
      {
         transformToLocal(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
      }

      @Override
      public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
      {
         transformToLocal(matrixOriginal, matrixTransformed);
      }

      @Override
      public void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
      {
         transformToLocal(matrixOriginal, matrixTransformed);
      }

      @Override
      public void inverseTransform(RigidBodyTransform original, RigidBodyTransform transformed)
      {
         transformToLocal(original, transformed);
      }

      @Override
      public void inverseTransform(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
      {
         transformToLocal(original, transformed);
      }

      @Override
      public void inverseTransform(AffineTransform original, AffineTransform transformed)
      {
         throw new RuntimeException("Affine transforms can't be transformed to Pose frame.");
      }
   }
}
