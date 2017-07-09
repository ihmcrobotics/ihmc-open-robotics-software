package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class RawEncoderTicks implements AcsellSlowSensor
{
   private final YoInteger rawEncoderTicks;
   
   public RawEncoderTicks(String name, YoVariableRegistry registry)
   {
      rawEncoderTicks = new YoInteger(name + "RawEncoderADTicks", registry);
   }

   @Override
   public void update(int value)
   {
      rawEncoderTicks.set(value);
   }

}
