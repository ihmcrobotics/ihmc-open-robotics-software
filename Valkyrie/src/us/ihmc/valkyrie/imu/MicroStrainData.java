package us.ihmc.valkyrie.imu;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.concurrent.Builder;

public class MicroStrainData
{
   public enum MicrostrainFilterType
   {ADAPTIVE_EKF, COMPLIMENTARY_FILTER}
   
   public static final Matrix3d MICROSTRAIN_TO_ZUP_WORLD = new Matrix3d(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
   public static final double MICROSTRAIN_GRAVITY = 9.80665; // MicroStrain's definition of g, as given on pg 64 of '3DM-GX3-15-25 MIP Data Communications Protocol'
   
   private long receiveTime;
   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularRate = new Vector3d();
   private final Quat4d quaternion = new Quat4d();
   private final Matrix3d orientationMatrix = new Matrix3d();
   
   private boolean isAccelerationValid = false;
   private boolean isAngularRateValid = false;
   private boolean isQuaternionValid = false;
   private boolean isMatrixValid = false;
   
   private MicrostrainFilterType filterType;

   public MicrostrainFilterType getFilterType()
   {
      return filterType;
   }

   public void setFilterType(MicrostrainFilterType filterType)
   {
      this.filterType = filterType;
   }

   public void setReceiveTime(long time)
   {
      receiveTime = time;
   }
   
   public void setLinearAcceleration(double x, double y, double z)
   {
      linearAcceleration.set(x, y, z);
   }

   public void setAngularRate(double wx, double wy, double wz)
   {
      angularRate.set(wx, wy, wz);
   }

   public void setQuaternion(double q0, double q1, double q2, double q3)
   {
      quaternion.set(q1, q2, q3, q0);
   }

   public Vector3d getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public Vector3d getAngularRate()
   {
      return angularRate;
   }

   public Quat4d getQuaternion()
   {
      return quaternion;
   }
   
   public Matrix3d getOrientationMatrix()
   {
      return orientationMatrix;
   }
   
   public long getReceiveTime()
   {
      return receiveTime;
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("MicroStrainData [acceleration=").append(linearAcceleration).append(", gyro=").append(angularRate).append(", quaternion=").append(quaternion)
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

   public void setOrientationMatrix(float m0, float m1, float m2, float m3, float m4, float m5, float m6, float m7, float m8)
   {
      orientationMatrix.setM00(m0);
      orientationMatrix.setM10(m1);
      orientationMatrix.setM20(m2);
      orientationMatrix.setM01(m3);
      orientationMatrix.setM11(m4);
      orientationMatrix.setM21(m5);
      orientationMatrix.setM02(m6);
      orientationMatrix.setM12(m7);
      orientationMatrix.setM22(m8);
   }

   public boolean isLinearAccelerationValid()
   {
      return isAccelerationValid;
   }

   public void setAccelerationValid(boolean isAccelerationValid)
   {
      this.isAccelerationValid = isAccelerationValid;
   }

   public boolean isAngularRateValid()
   {
      return isAngularRateValid;
   }

   public void setAngularRateValid(boolean isAngularRateValid)
   {
      this.isAngularRateValid = isAngularRateValid;
   }

   public boolean isQuaternionValid()
   {
      return isQuaternionValid;
   }

   public void setQuaternionValid(boolean isQuaternionValid)
   {
      this.isQuaternionValid = isQuaternionValid;
   }

   public boolean isMatrixValid()
   {
      return isMatrixValid;
   }

   public void setMatrixValid(boolean isMatrixValid)
   {
      this.isMatrixValid = isMatrixValid;
   }
}
