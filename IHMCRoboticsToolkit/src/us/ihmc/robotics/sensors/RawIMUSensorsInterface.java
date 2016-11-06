package us.ihmc.robotics.sensors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public interface RawIMUSensorsInterface
{
   public abstract void setOrientation(Matrix3d orientation, int imuIndex);

   public abstract void setAcceleration(Vector3d acceleration, int imuIndex);

   public abstract void setAngularVelocity(Vector3d angularVelocity, int imuIndex);

   public abstract void setCompass(Vector3d compass, int imuIndex);

   public abstract void getOrientation(Matrix3d orientationToPack, int imuIndex);

   public abstract void getAcceleration(Vector3d accelerationToPack, int imuIndex);

   public abstract void getAngularVelocity(Vector3d angularVelocityToPack, int imuIndex);

   public abstract void getCompass(Vector3d compassToPack, int imuIndex);
}
