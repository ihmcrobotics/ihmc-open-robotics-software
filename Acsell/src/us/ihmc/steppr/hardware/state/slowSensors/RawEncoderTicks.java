package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class RawEncoderTicks implements StepprSlowSensor
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
