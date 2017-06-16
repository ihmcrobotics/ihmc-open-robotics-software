package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class IMUGyroSensor implements AcsellSlowSensor
{
   private final YoDouble imuGyro;
   
   public IMUGyroSensor(String name, String axis, YoVariableRegistry registry)
   {
      imuGyro = new YoDouble(name + "IMUGyro" + axis, registry);
   }

   @Override
   public void update(int value)
   {
      imuGyro.set(((short)value));
   }

}
