package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BusVoltage implements AcsellSlowSensor
{
   private final YoDouble voltage;
   private final double conversionFactor;

   public BusVoltage(String name, double conversionFactor, YoVariableRegistry slowSensorRegistry)
   {
      voltage = new YoDouble(name + "BusVoltage", slowSensorRegistry);
      this.conversionFactor = conversionFactor;
   }

   @Override
   public void update(int value)
   {
      voltage.set(((double) value) * conversionFactor); //3.3 / 4096 * 100.0 / 2.2);
   }
}
