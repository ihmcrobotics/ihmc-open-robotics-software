package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class InphaseControlEffort implements StepprSlowSensor
{
   private final DoubleYoVariable inphaseControlEffort;
   
   public InphaseControlEffort(String name, YoVariableRegistry registry)
   {
      inphaseControlEffort = new DoubleYoVariable(name + "InphaseControlEffort", registry);
   }

   @Override
   public void update(int value)
   {
      inphaseControlEffort.set(((double)((short) value)) / 100.0);
   }
   
   public double getValue()
   {
	   return inphaseControlEffort.getDoubleValue();
   }

}
