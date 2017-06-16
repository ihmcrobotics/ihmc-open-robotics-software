package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class IMUMagSensor implements AcsellSlowSensor
{
   private final YoDouble imuMag;
   
   public IMUMagSensor(String name, String axis, YoVariableRegistry registry)
   {
      imuMag = new YoDouble(name + "IMUMag" + axis, registry);
   }

   @Override
   public void update(int value)
   {
      imuMag.set(((short)value));
   }

   public double getValue()
   {
      return imuMag.getDoubleValue();
   }

}
