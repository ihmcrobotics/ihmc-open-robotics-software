package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.corruptors.NoisyDoubleYoVariable;
import us.ihmc.robotics.math.corruptors.NoisyYoRotationMatrix;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public abstract class SimulatedIMURawSensorReader implements RawSensorReader
{
   private final String name;
   protected final YoVariableRegistry registry;

   private final RawIMUSensorsInterface rawSensors;
   protected final int imuIndex;
   protected final RigidBody rigidBody;
   protected final ReferenceFrame imuFrame;

   private final FramePoint imuFramePoint;
   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame bodyFrame;

   private final RotationMatrix orientation = new RotationMatrix();
   private final FrameVector acceleration = new FrameVector(worldFrame);
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D compass = new Vector3D();

   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final Twist twist = new Twist();
   private final Twist twistInIMUFrame = new Twist();
   private final Twist twistInWorldFrame = new Twist();
   private final SpatialAccelerationVector spatialAcceleration;

   protected final DoubleYoVariable perfM00;
   protected final DoubleYoVariable perfM01;
   protected final DoubleYoVariable perfM02;

   protected final DoubleYoVariable perfM10;
   protected final DoubleYoVariable perfM11;
   protected final DoubleYoVariable perfM12;

   protected final DoubleYoVariable perfM20;
   protected final DoubleYoVariable perfM21;
   protected final DoubleYoVariable perfM22;

   protected final DoubleYoVariable perfAccelX;
   protected final DoubleYoVariable perfAccelY;
   protected final DoubleYoVariable perfAccelZ;

   protected final DoubleYoVariable perfGyroX;
   protected final DoubleYoVariable perfGyroY;
   protected final DoubleYoVariable perfGyroZ;

   protected final DoubleYoVariable perfCompassX;
   protected final DoubleYoVariable perfCompassY;
   protected final DoubleYoVariable perfCompassZ;
   
   protected final NoisyYoRotationMatrix rotationMatrix;

   protected final NoisyDoubleYoVariable accelX;
   protected final NoisyDoubleYoVariable accelY;
   protected final NoisyDoubleYoVariable accelZ;

   protected final NoisyDoubleYoVariable gyroX;
   protected final NoisyDoubleYoVariable gyroY;
   protected final NoisyDoubleYoVariable gyroZ;

   protected final NoisyDoubleYoVariable compassX;
   protected final NoisyDoubleYoVariable compassY;
   protected final NoisyDoubleYoVariable compassZ;
   
   protected final NoisyDoubleYoVariable[] accelList;
   protected final NoisyDoubleYoVariable[] gyroList;
   protected final NoisyDoubleYoVariable[] compassList;

   public SimulatedIMURawSensorReader(RawIMUSensorsInterface rawSensors, int imuIndex, RigidBody rigidBody, ReferenceFrame imuFrame, RigidBody rootBody, SpatialAccelerationVector rootAcceleration)
   {
      this.rawSensors = rawSensors;
      this.imuIndex = imuIndex;
      this.rigidBody = rigidBody;
      this.imuFrame = imuFrame;
      this.twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootBody);
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, ReferenceFrame.getWorldFrame(), rootAcceleration, twistCalculator, true, false);

      name = getClass().getSimpleName() + imuIndex;
      registry = new YoVariableRegistry(name);

      imuFramePoint = new FramePoint(imuFrame);
      bodyFrame = rigidBody.getBodyFixedFrame();

      spatialAcceleration = new SpatialAccelerationVector(bodyFrame, worldFrame, bodyFrame);

      perfM00 = new DoubleYoVariable("perf_imu_m00", registry);
      perfM01 = new DoubleYoVariable("perf_imu_m01", registry);
      perfM02 = new DoubleYoVariable("perf_imu_m02", registry);

      perfM10 = new DoubleYoVariable("perf_imu_m10", registry);
      perfM11 = new DoubleYoVariable("perf_imu_m11", registry);
      perfM12 = new DoubleYoVariable("perf_imu_m12", registry);

      perfM20 = new DoubleYoVariable("perf_imu_m20", registry);
      perfM21 = new DoubleYoVariable("perf_imu_m21", registry);
      perfM22 = new DoubleYoVariable("perf_imu_m22", registry);

      perfAccelX = new DoubleYoVariable("perf_imu_accel_x", registry);
      perfAccelY = new DoubleYoVariable("perf_imu_accel_y", registry);
      perfAccelZ = new DoubleYoVariable("perf_imu_accel_z", registry);

      perfGyroX = new DoubleYoVariable("perf_imu_gyro_x", registry);
      perfGyroY = new DoubleYoVariable("perf_imu_gyro_y", registry);
      perfGyroZ = new DoubleYoVariable("perf_imu_gyro_z", registry);

      perfCompassX = new DoubleYoVariable("perf_imu_compass_x", registry);
      perfCompassY = new DoubleYoVariable("perf_imu_compass_y", registry);
      perfCompassZ = new DoubleYoVariable("perf_imu_compass_z", registry);
      
      rotationMatrix = new NoisyYoRotationMatrix("r_imu", registry);
      
      accelX = new NoisyDoubleYoVariable("r_imu_accel_x", registry, perfAccelX);
      accelY = new NoisyDoubleYoVariable("r_imu_accel_y", registry, perfAccelY);
      accelZ = new NoisyDoubleYoVariable("r_imu_accel_z", registry, perfAccelZ);

      gyroX = new NoisyDoubleYoVariable("r_imu_gyro_x", registry, perfGyroX);
      gyroY = new NoisyDoubleYoVariable("r_imu_gyro_y", registry, perfGyroY);
      gyroZ = new NoisyDoubleYoVariable("r_imu_gyro_z", registry, perfGyroZ);

      compassX = new NoisyDoubleYoVariable("r_imu_compass_x", registry, perfCompassX);
      compassY = new NoisyDoubleYoVariable("r_imu_compass_y", registry, perfCompassY);
      compassZ = new NoisyDoubleYoVariable("r_imu_compass_z", registry, perfCompassZ);
      
      accelList = new NoisyDoubleYoVariable[]{accelX, accelY, accelZ};
      gyroList = new NoisyDoubleYoVariable[]{gyroX, gyroY, gyroZ};
      compassList = new NoisyDoubleYoVariable[]{compassX, compassY, compassZ};
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
      spatialAccelerationCalculator.compute();
      twistCalculator.getTwistOfBody(rigidBody, twist);    // Twist of bodyCoM and not IMU!
      spatialAccelerationCalculator.getAccelerationOfBody(rigidBody, spatialAcceleration);
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
      rawSensors.setAcceleration(acceleration.getVectorCopy(), imuIndex);
      rawSensors.setAngularVelocity(angularVelocity, imuIndex);
      rawSensors.setCompass(compass, imuIndex);
   }

   protected void updatePerfectOrientation()
   {
      imuFrame.getTransformToDesiredFrame(worldFrame).getRotation(orientation);

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
      twistInIMUFrame.set(twist);
      twistInIMUFrame.changeFrame(imuFrame);
      twistInIMUFrame.getAngularPart(angularVelocity);

      perfGyroX.set(angularVelocity.getX());
      perfGyroY.set(angularVelocity.getY());
      perfGyroZ.set(angularVelocity.getZ());
   }

   protected void updatePerfectAcceleration()
   {
      twistInWorldFrame.set(twist);
      twistInWorldFrame.changeFrame(worldFrame);

      FramePoint imuFramePointInWorldFrame = new FramePoint(imuFramePoint);
      imuFramePointInWorldFrame.changeFrame(worldFrame);

      acceleration.setToZero(worldFrame);
      spatialAcceleration.getAccelerationOfPointFixedInBodyFrame(twistInWorldFrame, imuFramePointInWorldFrame, acceleration);
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
   public YoVariableRegistry getYoVariableRegistry()
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