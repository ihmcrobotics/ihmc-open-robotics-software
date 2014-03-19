package us.ihmc.valkyrie.imu;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.concurrent.Builder;

public class MicroStrainData
{
   public static final Matrix3d MICROSTRAIN_TO_ZUP_WORLD = new Matrix3d(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
   public static final double MICROSTRAIN_GRAVITY = 9.80665; // MicroStrain's definition of g, as given on pg 64 of '3DM-GX3-15-25 MIP Data Communications Protocol'
   
   private long receiveTime;
   private final Vector3d acceleration = new Vector3d();
   private final Vector3d gyro = new Vector3d();
   private final Quat4d quaternion = new Quat4d();
   private final Matrix3d matrix = new Matrix3d();

   public void setReceiveTime(long time)
   {
      receiveTime = time;
   }
   
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
   
   public Matrix3d getMatrix()
   {
      return matrix;
   }
   
   public long getReceiveTime()
   {
      return receiveTime;
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

   public void setMatrix(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8)
   {
      matrix.setM00(m0);
      matrix.setM10(m1);
      matrix.setM20(m2);
      matrix.setM01(m3);
      matrix.setM11(m4);
      matrix.setM21(m5);
      matrix.setM02(m6);
      matrix.setM12(m7);
      matrix.setM22(m8);
   }
}
