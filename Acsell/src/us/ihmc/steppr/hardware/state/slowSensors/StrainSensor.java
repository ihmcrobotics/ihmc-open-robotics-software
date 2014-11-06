package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StrainSensor implements StepprSlowSensor
{
   private final DoubleYoVariable strainSensor;
   
   public StrainSensor(String name, int sensor, YoVariableRegistry registry)
   {
      strainSensor = new DoubleYoVariable(name + "StrainSensor" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      strainSensor.set(value);
   }

}
