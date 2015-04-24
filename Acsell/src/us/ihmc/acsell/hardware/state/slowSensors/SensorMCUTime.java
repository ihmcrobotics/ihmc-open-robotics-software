package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class SensorMCUTime implements AcsellSlowSensor
{
   private final IntegerYoVariable time;
   
   public SensorMCUTime(String name, YoVariableRegistry slowSensorRegistry)
   {
      time = new IntegerYoVariable(name + "SensorMCUTime", slowSensorRegistry);
   }

   @Override
   public void update(int value)
   {
      time.set(value);
   }

}
