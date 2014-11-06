package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class PressureSensor implements StepprSlowSensor
{
   private final DoubleYoVariable pressureSensor;
   
   public PressureSensor(String name, int sensor, YoVariableRegistry registry)
   {
      pressureSensor = new DoubleYoVariable(name + "PressureSensor" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      pressureSensor.set(value);
   }

}
