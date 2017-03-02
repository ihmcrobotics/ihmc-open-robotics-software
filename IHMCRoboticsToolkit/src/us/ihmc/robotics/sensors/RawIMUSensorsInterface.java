package us.ihmc.robotics.sensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;

public interface RawIMUSensorsInterface
{
   public abstract void setOrientation(RotationMatrix orientation, int imuIndex);

   public abstract void setAcceleration(Vector3D acceleration, int imuIndex);

   public abstract void setAngularVelocity(Vector3D angularVelocity, int imuIndex);

   public abstract void setCompass(Vector3D compass, int imuIndex);

   public abstract void getOrientation(RotationMatrix orientationToPack, int imuIndex);

   public abstract void getAcceleration(Vector3D accelerationToPack, int imuIndex);

   public abstract void getAngularVelocity(Vector3D angularVelocityToPack, int imuIndex);

   public abstract void getCompass(Vector3D compassToPack, int imuIndex);
}
