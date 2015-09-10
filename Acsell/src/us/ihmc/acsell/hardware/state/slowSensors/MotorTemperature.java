package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class MotorTemperature implements AcsellSlowSensor
{
   private final DoubleYoVariable motorTemperature;
   private final double conversionFactor;
   
   public MotorTemperature(String name, double conversionFactor, YoVariableRegistry parentRegistry)
   {
      motorTemperature = new DoubleYoVariable(name + "MotorTemperature", parentRegistry);
      this.conversionFactor = conversionFactor;
   }

   @Override
   public void update(int value)
   {
      motorTemperature.set(((double)value)/conversionFactor);// /100.0);
   }
   
}
