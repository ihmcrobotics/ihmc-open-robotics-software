package us.ihmc.sensorProcessing.signalCorruption;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RotationFunctions;


public class OrientationConstantAcceleratingYawDriftCorruptor implements SignalCorruptor<Matrix3d>
{
   public static final String SIMULATED_YAW_DRIFT_ACCELERATION = "SimulatedYawDriftAcceleration";

   private final YoVariableRegistry registry;

   private final DoubleYoVariable yawDriftAcceleration;
   private final DoubleYoVariable yawDriftVelocity;
   private final DoubleYoVariable yawDriftAngle;
   private final DoubleYoVariable corruptedIMUYawAngle;

   private final Matrix3d yawDriftRotation = new Matrix3d();
   private final Matrix3d tempRotation = new Matrix3d();

   private final double dt;

   public OrientationConstantAcceleratingYawDriftCorruptor(String namePrefix, double dt, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.dt = dt;

      this.yawDriftAcceleration = new DoubleYoVariable(namePrefix + SIMULATED_YAW_DRIFT_ACCELERATION, registry);
      this.yawDriftVelocity = new DoubleYoVariable(namePrefix + "SimulatedYawDriftVelocity", registry);
      this.yawDriftAngle = new DoubleYoVariable(namePrefix + "SimulatedYawDriftAngle", registry);
      this.corruptedIMUYawAngle = new DoubleYoVariable(namePrefix + "SimulatedCorruptedIMUYawAngle", registry);

      parentRegistry.addChild(registry);
   }
   
   public void setYawDriftAcceleration(double yawDriftAcceleration)
   {
      this.yawDriftAcceleration.set(yawDriftAcceleration);
   }

   public void corrupt(Matrix3d signal)
   {
      yawDriftVelocity.add(yawDriftAcceleration.getDoubleValue() * dt);
      yawDriftAngle.add(yawDriftVelocity.getDoubleValue() * dt);

      yawDriftRotation.rotZ(yawDriftAngle.getDoubleValue());

      tempRotation.mul(yawDriftRotation, signal);
      signal.set(tempRotation);
      
      corruptedIMUYawAngle.set(RotationFunctions.getYaw(tempRotation));
   }
}
