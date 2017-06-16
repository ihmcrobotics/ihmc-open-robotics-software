package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class MotorThermistorADTicks implements AcsellSlowSensor
{
   private final YoInteger motorTemperature;
   
   public MotorThermistorADTicks(String name, YoVariableRegistry parentRegistry)
   {
      motorTemperature = new YoInteger(name + "MotorThermisterADTicks", parentRegistry);
   }

   @Override
   public void update(int value)
   {
      motorTemperature.set(value);
   }
   
}
