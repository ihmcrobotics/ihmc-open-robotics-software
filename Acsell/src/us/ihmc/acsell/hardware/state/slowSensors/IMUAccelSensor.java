package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class IMUAccelSensor implements AcsellSlowSensor
{
   private final DoubleYoVariable imuAccel;
   
   public IMUAccelSensor(String name, String axis, YoVariableRegistry registry)
   {
      imuAccel = new DoubleYoVariable(name + "IMUAccel" + axis, registry);
   }

   @Override
   public void update(int value)
   {
      imuAccel.set(((short)value));
   }

   public double getValue()
   {
      return imuAccel.getDoubleValue();
   }

}
