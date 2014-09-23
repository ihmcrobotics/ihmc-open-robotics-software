package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class BusVoltage implements StepprSlowSensor
{
   private final DoubleYoVariable busVoltage;
   
   public BusVoltage(String name, YoVariableRegistry parentRegistry)
   {
      busVoltage = new DoubleYoVariable(name + "BusVoltage", parentRegistry);
   }

   @Override
   public void update(int value)
   {
      busVoltage.set(value);
   }
   
}
