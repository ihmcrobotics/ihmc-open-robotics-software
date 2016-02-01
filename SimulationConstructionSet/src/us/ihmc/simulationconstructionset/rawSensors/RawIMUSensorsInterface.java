package us.ihmc.simulationconstructionset.rawSensors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public interface RawIMUSensorsInterface
{
   public abstract void setOrientation(Matrix3d orientation, int imuIndex);

   public abstract void setAcceleration(Vector3d acceleration, int imuIndex);

   public abstract void setAngularVelocity(Vector3d angularVelocity, int imuIndex);

   public abstract void setCompass(Vector3d compass, int imuIndex);

   public abstract void packOrientation(Matrix3d orientationToPack, int imuIndex);

   public abstract void packAcceleration(Vector3d accelerationToPack, int imuIndex);

   public abstract void packAngularVelocity(Vector3d angularVelocityToPack, int imuIndex);

   public abstract void packCompass(Vector3d compassToPack, int imuIndex);
}
