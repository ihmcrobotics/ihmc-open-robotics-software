package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class IMUAccelSensor implements AcsellSlowSensor
{
   private final YoDouble imuAccel;
   
   public IMUAccelSensor(String name, String axis, YoVariableRegistry registry)
   {
      imuAccel = new YoDouble(name + "IMUAccel" + axis, registry);
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
