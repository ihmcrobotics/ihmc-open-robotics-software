package us.ihmc.robotics.geometry.transformables;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class Pose implements GeometryObject<Pose>
{
   private final Quaternion orientation;
   private final Point3D position;

   public Pose(Pose pose)
   {
      orientation = new Quaternion(pose.getOrientation());
      position = new Point3D(pose.getPosition());
   }

   public Pose()
   {
      orientation = new Quaternion();
      position = new Point3D();
   }

   public Pose(RigidBodyTransform transform)
   {
      transform.getRotation(orientation = new Quaternion());
      transform.getTranslation(position = new Point3D());
   }

   public Pose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation = new Quaternion(orientation);
      this.position = new Point3D(position);
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

   @Override
   public void set(Pose other)
   {
      orientation.set(other.getOrientation());
      position.set(other.getPosition());
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

   @Override
   public boolean containsNaN()
   {
      return orientation.containsNaN() || position.containsNaN();
   }

   @Override
   public boolean epsilonEquals(Pose other, double epsilon)
   {
      return orientation.epsilonEquals(other.getOrientation(), epsilon) && position.epsilonEquals(other.getPosition(), epsilon);
   }

   @Override
   public void applyTransform(Transform transform)
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
   
   public void interpolate(Pose pose1, Pose pose2, double alpha)
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

   public boolean epsilonEquals(Pose other, double positionErrorMargin, double orientationErrorMargin)
   {
      return position.epsilonEquals(other.getPosition(), positionErrorMargin) && orientation.epsilonEquals(other.getOrientation(), orientationErrorMargin);
   }
}
