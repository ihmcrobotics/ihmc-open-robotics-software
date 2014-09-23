package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class BusVoltage implements StepprSlowSensor
{
   private final DoubleYoVariable busVoltage;
   
   public BusVoltage(YoVariableRegistry parentRegistry)
   {
      busVoltage = new DoubleYoVariable("busVoltage", parentRegistry);
   }

   @Override
   public void update(int value)
   {
      busVoltage.set(value);
   }
   
}
