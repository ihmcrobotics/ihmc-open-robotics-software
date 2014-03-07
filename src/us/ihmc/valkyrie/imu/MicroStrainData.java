package us.ihmc.valkyrie.imu;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.concurrent.Builder;

public class MicroStrainData
{
   private final Vector3d acceleration = new Vector3d();
   private final Vector3d gyro = new Vector3d();
   private final Quat4d quaternion = new Quat4d();

   public void setAcceleration(double x, double y, double z)
   {
      acceleration.set(x, y, z);
   }

   public void setGyro(double wx, double wy, double wz)
   {
      gyro.set(wx, wy, wz);
   }

   public void setQuaternion(double q0, double q1, double q2, double q3)
   {
      quaternion.set(q1, q2, q3, q0);
   }

   public Vector3d getAcceleration()
   {
      return acceleration;
   }

   public Vector3d getGyro()
   {
      return gyro;
   }

   public Quat4d getQuaternion()
   {
      return quaternion;
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("MicroStrainData [acceleration=").append(acceleration).append(", gyro=").append(gyro).append(", quaternion=").append(quaternion)
            .append("]");
      return builder.toString();
   }

   public static class MicroStrainDataBuilder implements Builder<MicroStrainData>
   {

      @Override
      public MicroStrainData newInstance()
      {
         return new MicroStrainData();
      }

   }
}
