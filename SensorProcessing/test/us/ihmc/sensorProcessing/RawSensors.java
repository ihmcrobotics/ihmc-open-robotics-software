package us.ihmc.sensorProcessing;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class RawSensors implements RawIMUSensorsInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("RawSensors");

   private final DoubleYoVariable r_imu_m00;
   private final DoubleYoVariable r_imu_m01;
   private final DoubleYoVariable r_imu_m02;

   private final DoubleYoVariable r_imu_m10;
   private final DoubleYoVariable r_imu_m11;
   private final DoubleYoVariable r_imu_m12;

   private final DoubleYoVariable r_imu_m20;
   private final DoubleYoVariable r_imu_m21;
   private final DoubleYoVariable r_imu_m22;

   private final DoubleYoVariable r_imu_accel_x;
   private final DoubleYoVariable r_imu_accel_y;
   private final DoubleYoVariable r_imu_accel_z;

   private final DoubleYoVariable r_imu_gyro_x;
   private final DoubleYoVariable r_imu_gyro_y;
   private final DoubleYoVariable r_imu_gyro_z;

   private final DoubleYoVariable r_imu_compass_x;
   private final DoubleYoVariable r_imu_compass_y;
   private final DoubleYoVariable r_imu_compass_z;

   public final DoubleYoVariable r_time = new DoubleYoVariable("r_time", registry);

   public RawSensors(YoVariableRegistry yoVariableRegistry)
   {
      r_imu_m00 = new DoubleYoVariable("r_imu_m00", registry);
      r_imu_m01 = new DoubleYoVariable("r_imu_m01", registry);
      r_imu_m02 = new DoubleYoVariable("r_imu_m02", registry);

      r_imu_m10 = new DoubleYoVariable("r_imu_m10", registry);
      r_imu_m11 = new DoubleYoVariable("r_imu_m11", registry);
      r_imu_m12 = new DoubleYoVariable("r_imu_m12", registry);

      r_imu_m20 = new DoubleYoVariable("r_imu_m20", registry);
      r_imu_m21 = new DoubleYoVariable("r_imu_m21", registry);
      r_imu_m22 = new DoubleYoVariable("r_imu_m22", registry);

      r_imu_accel_x = new DoubleYoVariable("r_imu_accel_x", registry);
      r_imu_accel_y = new DoubleYoVariable("r_imu_accel_y", registry);
      r_imu_accel_z = new DoubleYoVariable("r_imu_accel_z", registry);

      r_imu_gyro_x = new DoubleYoVariable("r_imu_gyro_x", registry);
      r_imu_gyro_y = new DoubleYoVariable("r_imu_gyro_y", registry);
      r_imu_gyro_z = new DoubleYoVariable("r_imu_gyro_z", registry);

      r_imu_compass_x = new DoubleYoVariable("r_imu_compass_x", registry);
      r_imu_compass_y = new DoubleYoVariable("r_imu_compass_y", registry);
      r_imu_compass_z = new DoubleYoVariable("r_imu_compass_z", registry);

      yoVariableRegistry.addChild(registry);
   }

   public void setOrientation(RotationMatrix orientation, int imuIndex)
   {
      r_imu_m00.set(orientation.getM00());
      r_imu_m01.set(orientation.getM01());
      r_imu_m02.set(orientation.getM02());

      r_imu_m10.set(orientation.getM10());
      r_imu_m11.set(orientation.getM11());
      r_imu_m12.set(orientation.getM12());

      r_imu_m20.set(orientation.getM20());
      r_imu_m21.set(orientation.getM21());
      r_imu_m22.set(orientation.getM22());
   }

   public void setAcceleration(Vector3D acceleration, int imuIndex)
   {
      r_imu_accel_x.set(acceleration.getX());
      r_imu_accel_y.set(acceleration.getY());
      r_imu_accel_z.set(acceleration.getZ());
   }

   public void setAngularVelocity(Vector3D gyroscope, int imuIndex)
   {
      r_imu_gyro_x.set(gyroscope.getX());
      r_imu_gyro_y.set(gyroscope.getY());
      r_imu_gyro_z.set(gyroscope.getZ());
   }

   public void setCompass(Vector3D compass, int imuIndex)
   {
      r_imu_compass_x.set(compass.getX());
      r_imu_compass_y.set(compass.getY());
      r_imu_compass_z.set(compass.getZ());
   }

   public void getOrientation(RotationMatrix orientationToPack, int imuIndex)
   {
      orientationToPack.set(r_imu_m00.getDoubleValue(), r_imu_m01.getDoubleValue(), r_imu_m02.getDoubleValue(), r_imu_m10.getDoubleValue(),
                            r_imu_m11.getDoubleValue(), r_imu_m12.getDoubleValue(), r_imu_m20.getDoubleValue(), r_imu_m21.getDoubleValue(),
                            r_imu_m22.getDoubleValue());
   }

   public void getAcceleration(Vector3D accelerationToPack, int imuIndex)
   {
      accelerationToPack.set(r_imu_accel_x.getDoubleValue(), r_imu_accel_y.getDoubleValue(), r_imu_accel_z.getDoubleValue());
   }

   public void getAngularVelocity(Vector3D angularVelocityToPack, int imuIndex)
   {
      angularVelocityToPack.set(r_imu_gyro_x.getDoubleValue(), r_imu_gyro_y.getDoubleValue(), r_imu_gyro_z.getDoubleValue());
   }

   public void getCompass(Vector3D compassToPack, int imuIndex)
   {
      compassToPack.set(r_imu_compass_x.getDoubleValue(), r_imu_compass_y.getDoubleValue(), r_imu_compass_z.getDoubleValue());
   }
}
