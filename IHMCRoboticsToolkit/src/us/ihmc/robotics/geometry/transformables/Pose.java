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
   private final Point3D position;
   private final Quaternion orientation;

   public Pose(Pose pose)
   {
      this(pose.position, pose.orientation);
   }

   public Pose()
   {
      this.position = new Point3D();
      this.orientation = new Quaternion();
   }
   
   public Pose(RigidBodyTransform transform)
   {
      position = new Point3D();
      orientation = new Quaternion();
      
      setPose(transform);
   }

   public Pose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.position = new Point3D(position);
      this.orientation = new Quaternion(orientation);
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
      position.setX(position.getX() + x);
      position.setY(position.getY() + y);
      position.setZ(position.getZ() + z);
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
      this.position.set(other.position);
      this.orientation.set(other.orientation);
   }

   @Override
   public void setToZero()
   {
      this.position.setToZero();
      this.orientation.setToZero();
   }

   @Override
   public void setToNaN()
   {
      this.position.setToNaN();
      this.orientation.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      if (this.position.containsNaN())
         return true;
      if (this.orientation.containsNaN())
         return true;

      return false;
   }

   @Override
   public boolean epsilonEquals(Pose other, double epsilon)
   {
      return (this.position.epsilonEquals(other.position, epsilon)) && (this.orientation.epsilonEquals(other.orientation, epsilon));
   }

   @Override
   public void applyTransform(Transform transform)
   {
      this.position.applyTransform(transform);
      this.orientation.applyTransform(transform);
   }

   public void interpolate(Pose pose1, Pose pose2, double alpha)
   {
      position.interpolate(pose1.position, pose2.position, alpha);
      orientation.interpolate(pose1.orientation, pose2.orientation, alpha);
      orientation.normalize();
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
      transform.getTranslation(position);
      transform.getRotation(orientation);
   }

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setXY(Point2DReadOnly point)
   {
      setX(point.getX());
      setY(point.getY());     
   }
   
   public void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }
   
   public Point3D getPositionUnsafe()
   {
      return position;
   }
   
   public void getPosition(Tuple3DBasics tupleToPack)
   {
      tupleToPack.set(position); 
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      getPose(transformToPack);
   }

   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      this.orientation.set(qx, qy, qz, qs);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      this.orientation.set(quaternion);
   }

   public void setOrientation(RotationMatrixReadOnly matrix3d)
   {
      this.orientation.set(matrix3d);
   }

   public void setOrientation(AxisAngleReadOnly axisAngle4d)
   {
      this.orientation.set(axisAngle4d);
   }
   
   public Quaternion getOrientationUnsafe()
   {
      return orientation;
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
      this.orientation.setYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      this.orientation.getYawPitchRoll(yawPitchRollToPack);
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
      return (this.position.epsilonEquals(other.position, positionErrorMargin)) && (this.orientation.epsilonEquals(other.orientation, orientationErrorMargin));
   }


}
