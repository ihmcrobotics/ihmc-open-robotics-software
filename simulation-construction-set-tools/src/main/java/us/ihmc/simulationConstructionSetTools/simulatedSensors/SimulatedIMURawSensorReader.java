package us.ihmc.simulationConstructionSetTools.simulatedSensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.robotics.math.corruptors.NoisyYoDouble;
import us.ihmc.robotics.math.corruptors.NoisyYoRotationMatrix;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class SimulatedIMURawSensorReader implements RawSensorReader
{
   private final String name;
   protected final YoRegistry registry;

   private final RawIMUSensorsInterface rawSensors;
   protected final int imuIndex;
   protected final RigidBodyBasics rigidBody;
   protected final ReferenceFrame imuFrame;

   private final FramePoint3D imuFramePoint;
   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame bodyFrame;

   private final RotationMatrix orientation = new RotationMatrix();
   private final FrameVector3D acceleration = new FrameVector3D(worldFrame);
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D compass = new Vector3D();

   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final Twist twist = new Twist();
   private final Twist twistInIMUFrame = new Twist();
   private final Twist twistInWorldFrame = new Twist();
   private final SpatialAcceleration spatialAcceleration;

   protected final YoDouble perfM00;
   protected final YoDouble perfM01;
   protected final YoDouble perfM02;

   protected final YoDouble perfM10;
   protected final YoDouble perfM11;
   protected final YoDouble perfM12;

   protected final YoDouble perfM20;
   protected final YoDouble perfM21;
   protected final YoDouble perfM22;

   protected final YoDouble perfAccelX;
   protected final YoDouble perfAccelY;
   protected final YoDouble perfAccelZ;

   protected final YoDouble perfGyroX;
   protected final YoDouble perfGyroY;
   protected final YoDouble perfGyroZ;

   protected final YoDouble perfCompassX;
   protected final YoDouble perfCompassY;
   protected final YoDouble perfCompassZ;
   
   protected final NoisyYoRotationMatrix rotationMatrix;

   protected final NoisyYoDouble accelX;
   protected final NoisyYoDouble accelY;
   protected final NoisyYoDouble accelZ;

   protected final NoisyYoDouble gyroX;
   protected final NoisyYoDouble gyroY;
   protected final NoisyYoDouble gyroZ;

   protected final NoisyYoDouble compassX;
   protected final NoisyYoDouble compassY;
   protected final NoisyYoDouble compassZ;
   
   protected final NoisyYoDouble[] accelList;
   protected final NoisyYoDouble[] gyroList;
   protected final NoisyYoDouble[] compassList;

   public SimulatedIMURawSensorReader(RawIMUSensorsInterface rawSensors, int imuIndex, RigidBodyBasics rigidBody, ReferenceFrame imuFrame, RigidBodyBasics rootBody, SpatialAccelerationReadOnly rootAcceleration)
   {
      this.rawSensors = rawSensors;
      this.imuIndex = imuIndex;
      this.rigidBody = rigidBody;
      this.imuFrame = imuFrame;
      this.twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootBody);
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, ReferenceFrame.getWorldFrame());
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);

      name = getClass().getSimpleName() + imuIndex;
      registry = new YoRegistry(name);

      imuFramePoint = new FramePoint3D(imuFrame);
      bodyFrame = rigidBody.getBodyFixedFrame();

      spatialAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

      perfM00 = new YoDouble("perf_imu_m00", registry);
      perfM01 = new YoDouble("perf_imu_m01", registry);
      perfM02 = new YoDouble("perf_imu_m02", registry);

      perfM10 = new YoDouble("perf_imu_m10", registry);
      perfM11 = new YoDouble("perf_imu_m11", registry);
      perfM12 = new YoDouble("perf_imu_m12", registry);

      perfM20 = new YoDouble("perf_imu_m20", registry);
      perfM21 = new YoDouble("perf_imu_m21", registry);
      perfM22 = new YoDouble("perf_imu_m22", registry);

      perfAccelX = new YoDouble("perf_imu_accel_x", registry);
      perfAccelY = new YoDouble("perf_imu_accel_y", registry);
      perfAccelZ = new YoDouble("perf_imu_accel_z", registry);

      perfGyroX = new YoDouble("perf_imu_gyro_x", registry);
      perfGyroY = new YoDouble("perf_imu_gyro_y", registry);
      perfGyroZ = new YoDouble("perf_imu_gyro_z", registry);

      perfCompassX = new YoDouble("perf_imu_compass_x", registry);
      perfCompassY = new YoDouble("perf_imu_compass_y", registry);
      perfCompassZ = new YoDouble("perf_imu_compass_z", registry);
      
      rotationMatrix = new NoisyYoRotationMatrix("r_imu", registry);
      
      accelX = new NoisyYoDouble("r_imu_accel_x", registry, perfAccelX);
      accelY = new NoisyYoDouble("r_imu_accel_y", registry, perfAccelY);
      accelZ = new NoisyYoDouble("r_imu_accel_z", registry, perfAccelZ);

      gyroX = new NoisyYoDouble("r_imu_gyro_x", registry, perfGyroX);
      gyroY = new NoisyYoDouble("r_imu_gyro_y", registry, perfGyroY);
      gyroZ = new NoisyYoDouble("r_imu_gyro_z", registry, perfGyroZ);

      compassX = new NoisyYoDouble("r_imu_compass_x", registry, perfCompassX);
      compassY = new NoisyYoDouble("r_imu_compass_y", registry, perfCompassY);
      compassZ = new NoisyYoDouble("r_imu_compass_z", registry, perfCompassZ);
      
      accelList = new NoisyYoDouble[]{accelX, accelY, accelZ};
      gyroList = new NoisyYoDouble[]{gyroX, gyroY, gyroZ};
      compassList = new NoisyYoDouble[]{compassX, compassY, compassZ};
   }

   @Override
   public void initialize()
   {
      initializeNoise();
      read();
   }

   @Override
   public void read()
   {
      twistCalculator.compute();
      spatialAccelerationCalculator.reset();
      twistCalculator.getTwistOfBody(rigidBody, twist);    // Twist of bodyCoM and not IMU!
      spatialAcceleration.setIncludingFrame(spatialAccelerationCalculator.getAccelerationOfBody(rigidBody));
      spatialAcceleration.changeFrame(worldFrame, twist, twist);

      updatePerfectOrientation();
      updatePerfectAngularVelocity();
      updatePerfectAcceleration();
      updatePerfectCompass();

      simulateIMU();

      orientation.set(rotationMatrix.getMatrix3d());

      acceleration.set(accelX.getDoubleValue(), accelY.getDoubleValue(), accelZ.getDoubleValue());
      angularVelocity.set(gyroX.getDoubleValue(), gyroY.getDoubleValue(), gyroZ.getDoubleValue());
      compass.set(compassX.getDoubleValue(), compassY.getDoubleValue(), compassZ.getDoubleValue());

      rawSensors.setOrientation(orientation, imuIndex);
      rawSensors.setAcceleration(new Vector3D(acceleration), imuIndex);
      rawSensors.setAngularVelocity(angularVelocity, imuIndex);
      rawSensors.setCompass(compass, imuIndex);
   }

   protected void updatePerfectOrientation()
   {
      orientation.set(imuFrame.getTransformToDesiredFrame(worldFrame).getRotation());

      perfM00.set(orientation.getM00());
      perfM01.set(orientation.getM01());
      perfM02.set(orientation.getM02());

      perfM10.set(orientation.getM10());
      perfM11.set(orientation.getM11());
      perfM12.set(orientation.getM12());

      perfM20.set(orientation.getM20());
      perfM21.set(orientation.getM21());
      perfM22.set(orientation.getM22());
   }

   protected void updatePerfectAngularVelocity()
   {
      twistInIMUFrame.setIncludingFrame(twist);
      twistInIMUFrame.changeFrame(imuFrame);
      angularVelocity.set(twistInIMUFrame.getAngularPart());

      perfGyroX.set(angularVelocity.getX());
      perfGyroY.set(angularVelocity.getY());
      perfGyroZ.set(angularVelocity.getZ());
   }

   protected void updatePerfectAcceleration()
   {
      twistInWorldFrame.setIncludingFrame(twist);
      twistInWorldFrame.changeFrame(worldFrame);

      FramePoint3D imuFramePointInWorldFrame = new FramePoint3D(imuFramePoint);
      imuFramePointInWorldFrame.changeFrame(worldFrame);

      acceleration.setToZero(worldFrame);
      spatialAcceleration.getLinearAccelerationAt(twistInWorldFrame, imuFramePointInWorldFrame, acceleration);
      acceleration.changeFrame(imuFrame);

      perfAccelX.set(acceleration.getX());
      perfAccelY.set(acceleration.getY());
      perfAccelZ.set(acceleration.getZ());
   }

   protected void updatePerfectCompass()
   {
      // TODO
   }

   protected abstract void initializeNoise();
   
   protected abstract void simulateIMU();

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }
}