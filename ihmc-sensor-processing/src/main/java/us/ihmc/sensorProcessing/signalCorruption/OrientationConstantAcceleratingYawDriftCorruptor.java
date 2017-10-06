package us.ihmc.sensorProcessing.signalCorruption;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OrientationConstantAcceleratingYawDriftCorruptor implements SignalCorruptor<RotationMatrix>
{
   public static final String SIMULATED_YAW_DRIFT_ACCELERATION = "SimulatedYawDriftAcceleration";

   private final YoVariableRegistry registry;

   private final YoDouble yawDriftAcceleration;
   private final YoDouble yawDriftVelocity;
   private final YoDouble yawDriftAngle;
   private final YoDouble corruptedIMUYawAngle;

   private final RotationMatrix yawDriftRotation = new RotationMatrix();
   private final RotationMatrix tempRotation = new RotationMatrix();

   private final double dt;

   public OrientationConstantAcceleratingYawDriftCorruptor(String namePrefix, double dt, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.dt = dt;

      this.yawDriftAcceleration = new YoDouble(namePrefix + SIMULATED_YAW_DRIFT_ACCELERATION, registry);
      this.yawDriftVelocity = new YoDouble(namePrefix + "SimulatedYawDriftVelocity", registry);
      this.yawDriftAngle = new YoDouble(namePrefix + "SimulatedYawDriftAngle", registry);
      this.corruptedIMUYawAngle = new YoDouble(namePrefix + "SimulatedCorruptedIMUYawAngle", registry);

      parentRegistry.addChild(registry);
   }
   
   public void setYawDriftAcceleration(double yawDriftAcceleration)
   {
      this.yawDriftAcceleration.set(yawDriftAcceleration);
   }

   public void corrupt(RotationMatrix signal)
   {
      yawDriftVelocity.add(yawDriftAcceleration.getDoubleValue() * dt);
      yawDriftAngle.add(yawDriftVelocity.getDoubleValue() * dt);

      yawDriftRotation.setToYawMatrix(yawDriftAngle.getDoubleValue());

      tempRotation.set(yawDriftRotation);
      tempRotation.multiply(signal);
      signal.set(tempRotation);
      
      corruptedIMUYawAngle.set(tempRotation.getYaw());
   }
}
