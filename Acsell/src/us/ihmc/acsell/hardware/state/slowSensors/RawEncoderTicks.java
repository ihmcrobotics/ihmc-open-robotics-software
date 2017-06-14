package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.IntegerYoVariable;

public class RawEncoderTicks implements AcsellSlowSensor
{
   private final IntegerYoVariable rawEncoderTicks;
   
   public RawEncoderTicks(String name, YoVariableRegistry registry)
   {
      rawEncoderTicks = new IntegerYoVariable(name + "RawEncoderADTicks", registry);
   }

   @Override
   public void update(int value)
   {
      rawEncoderTicks.set(value);
   }

}
