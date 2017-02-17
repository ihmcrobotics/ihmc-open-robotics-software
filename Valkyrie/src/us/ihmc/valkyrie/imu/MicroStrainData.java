package us.ihmc.valkyrie.imu;

import us.ihmc.concurrent.Builder;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class MicroStrainData
{
   public enum MicrostrainFilterType
   {ADAPTIVE_EKF, COMPLIMENTARY_FILTER}
   
   public static final RotationMatrix MICROSTRAIN_TO_ZUP_WORLD = new RotationMatrix(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
   public static final double MICROSTRAIN_GRAVITY = 9.80665; // MicroStrain's definition of g, as given on pg 64 of '3DM-GX3-15-25 MIP Data Communications Protocol'
   
   private long receiveTime;
   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularRate = new Vector3D();
   private final Quaternion quaternion = new Quaternion();
   private final RotationMatrix orientationMatrix = new RotationMatrix();
   
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

   public Vector3D getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public Vector3D getAngularRate()
   {
      return angularRate;
   }

   public Quaternion getQuaternion()
   {
      return quaternion;
   }
   
   public RotationMatrix getOrientationMatrix()
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
      orientationMatrix.set(m0, m3, m6, m1, m4, m7, m2, m5, m8);
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
