package us.ihmc.valkyrieRosControl.dataHolders;

import java.io.IOException;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import com.esotericsoftware.minlog.Log;

import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.valkyrie.configuration.ValkyriePriorityParameters;
import us.ihmc.valkyrie.imu.MicroStrainData;
import us.ihmc.valkyrie.imu.MicrostrainUDPPacketListener;

public class MicroStrainIMUHandle implements IMUHandle
{

   private final String name;

   private final MicrostrainUDPPacketListener imuListener;
   private MicroStrainData microStrainData;

   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Quat4d orientation = new Quat4d();

   private Matrix3d quaternionConversionMatrix = new Matrix3d();
   private final Matrix3d orientationMatrix = new Matrix3d();

   /* package-private */ MicroStrainIMUHandle(String name, Integer id)
   {

      this.name = name;

      Log.info("Starting listener for IMU " + name);
      try
      {
         imuListener = MicrostrainUDPPacketListener.createRealtimeListener(ValkyriePriorityParameters.IMU_PRIORITY, id);
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot create listener for IMU " + name, e);
      }
   }

   public void update()
   {
      microStrainData = imuListener.getLatestData();

      if(microStrainData != null)
      {
	      linearAcceleration.set(microStrainData.getAcceleration());
	      linearAcceleration.scale(MicroStrainData.MICROSTRAIN_GRAVITY);
	
	      angularVelocity.set(microStrainData.getGyro());
	
	      quaternionConversionMatrix.set(microStrainData.getQuaternion());
	      orientationMatrix.mul(MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD, quaternionConversionMatrix);
	      RotationTools.setQuaternionBasedOnMatrix3d(orientation, orientationMatrix);
//	      orientation.set(microStrainData.getQuaternion());
      }
   }

   @Override
   public void getLinearAccelerationCovariance(double[] linearAccelerationCovariance)
   {

   }

   @Override
   public double getZdd()
   {
      return linearAcceleration.getZ();
   }

   @Override
   public double getYdd()
   {
      return linearAcceleration.getY();
   }

   @Override
   public double getXdd()
   {
      return linearAcceleration.getX();
   }

   @Override
   public void getAngularVelocityCovariance(double[] angularVelocityCovariance)
   {

   }

   @Override
   public double getTheta_z()
   {
      return angularVelocity.getZ();
   }

   @Override
   public double getTheta_y()
   {
      return angularVelocity.getY();
   }

   @Override
   public double getTheta_x()
   {
      return angularVelocity.getX();
   }

   @Override
   public void getOrientationCovariance(double[] orientationCovariance)
   {
   }

   @Override
   public double getQ_w()
   {
      return orientation.getW();
   }

   @Override
   public double getQ_z()
   {
      return orientation.getZ();
   }

   @Override
   public double getQ_y()
   {
      return orientation.getY();
   }

   @Override
   public double getQ_x()
   {
      return orientation.getX();
   }

   @Override
   public String getName()
   {
      return name;
   }

}
