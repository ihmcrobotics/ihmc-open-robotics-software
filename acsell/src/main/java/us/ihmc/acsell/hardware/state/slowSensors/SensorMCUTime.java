package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class SensorMCUTime implements AcsellSlowSensor
{
   private final YoInteger time;
   
   public SensorMCUTime(String name, YoVariableRegistry slowSensorRegistry)
   {
      time = new YoInteger(name + "SensorMCUTime", slowSensorRegistry);
   }

   @Override
   public void update(int value)
   {
      time.set(value);
   }

}
