package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class BusVoltage implements AcsellSlowSensor
{
   private final DoubleYoVariable voltage;

   public BusVoltage(String name, YoVariableRegistry slowSensorRegistry)
   {
      voltage = new DoubleYoVariable(name + "BusVoltage", slowSensorRegistry);
   }

   @Override
   public void update(int value)
   {
      voltage.set(((double) value) * 3.3 / 4096 * 100.0 / 2.2);
   }
}
