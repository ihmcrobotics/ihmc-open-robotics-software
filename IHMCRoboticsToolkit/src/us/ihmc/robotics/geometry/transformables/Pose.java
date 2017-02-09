package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class Pose implements GeometryObject<Pose>
{
   private final TransformablePoint3d position;
   private final TransformableQuat4d orientation;

   public Pose(Pose pose)
   {
      this(pose.position, pose.orientation);
   }

   public Pose()
   {
      this.position = new TransformablePoint3d();
      this.orientation = new TransformableQuat4d();
   }
   
   public Pose(Point3d position, Quat4d orientation)
   {
      this.position = new TransformablePoint3d(position);
      this.orientation = new TransformableQuat4d(orientation);
   }
   
   public Pose(Point3f position, Quat4f orientation)
   {
      this.position = new TransformablePoint3d(position);
      this.orientation = new TransformableQuat4d(orientation);
   }
   
   public Pose(RigidBodyTransform transform)
   {
      position = new TransformablePoint3d();
      orientation = new TransformableQuat4d();
      
      setPose(transform);
   }

   public Pose(TransformablePoint3d position, TransformableQuat4d orientation)
   {
      this.position = new TransformablePoint3d(position);
      this.orientation = new TransformableQuat4d(orientation);
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

   public void translate(Tuple3d translation)
   {
      position.add(translation);
   }

   public void translate(double x, double y, double z)
   {
      position.setX(position.getX() + x);
      position.setY(position.getY() + y);
      position.setZ(position.getZ() + z);
   }

   public TransformablePoint3d getPoint()
   {
      return position;
   }

   public TransformableQuat4d getOrientation()
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
   public void applyTransform(RigidBodyTransform transform)
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

   public void setPose(Tuple3d position, Quat4d orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Tuple3f position, Quat4f orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Tuple3d position, AxisAngle4d orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPosition(Tuple3d position)
   {
      this.position.set(position);
   }

   public void setPosition(Tuple3f position)
   {
      this.position.set(position);
   }

   public void setXY(Point2d point)
   {
      setX(point.getX());
      setY(point.getY());     
   }
   
   public void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(orientation, position);
   }
   
   public TransformablePoint3d getPositionUnsafe()
   {
      return position;
   }
   
   public void getPosition(Tuple3d tupleToPack)
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

   public void setOrientation(Quat4d quaternion)
   {
      this.orientation.setOrientation(quaternion);
   }

   public void setOrientation(Quat4f quaternion)
   {
      this.orientation.setOrientation(quaternion);
   }
   
   public void setOrientation(Matrix3d matrix3d)
   {
      this.orientation.setOrientation(matrix3d);
   }

   public void setOrientation(AxisAngle4d axisAngle4d)
   {
      this.orientation.setOrientation(axisAngle4d);
   }
   
   public TransformableQuat4d getOrientationUnsafe()
   {
      return orientation;
   }

   public void getOrientation(Matrix3d matrixToPack)
   {
      matrixToPack.set(orientation);    
   }

   public void getOrientation(Quat4d quaternionToPack)
   {
      quaternionToPack.set(orientation);
   }
   
   public void getOrientation(AxisAngle4d axisAngleToPack)
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
