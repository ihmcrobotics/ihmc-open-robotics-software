package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MotorTemperature implements AcsellSlowSensor
{
   private final YoDouble motorTemperature;
   private final double conversionFactor;
   
   public MotorTemperature(String name, double conversionFactor, YoVariableRegistry parentRegistry)
   {
      motorTemperature = new YoDouble(name + "MotorTemperature", parentRegistry);
      this.conversionFactor = conversionFactor;
   }

   @Override
   public void update(int value)
   {
      motorTemperature.set(((double)value)/conversionFactor);// /100.0);
   }
   
}
