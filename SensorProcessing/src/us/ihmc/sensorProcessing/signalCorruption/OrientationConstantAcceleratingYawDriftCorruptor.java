package us.ihmc.sensorProcessing.signalCorruption;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class OrientationConstantAcceleratingYawDriftCorruptor implements SignalCorruptor<RotationMatrix>
{
   public static final String SIMULATED_YAW_DRIFT_ACCELERATION = "SimulatedYawDriftAcceleration";

   private final YoVariableRegistry registry;

   private final DoubleYoVariable yawDriftAcceleration;
   private final DoubleYoVariable yawDriftVelocity;
   private final DoubleYoVariable yawDriftAngle;
   private final DoubleYoVariable corruptedIMUYawAngle;

   private final RotationMatrix yawDriftRotation = new RotationMatrix();
   private final RotationMatrix tempRotation = new RotationMatrix();

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
