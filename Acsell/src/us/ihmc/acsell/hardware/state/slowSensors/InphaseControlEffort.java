package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class InphaseControlEffort implements AcsellSlowSensor
{
   private final DoubleYoVariable inphaseControlEffort;
   private final double conversionFactor;
   
   public InphaseControlEffort(String name, double conversionFactor, YoVariableRegistry registry)
   {
      inphaseControlEffort = new DoubleYoVariable(name + "InphaseControlEffort", registry);
      this.conversionFactor = conversionFactor;
   }

   @Override
   public void update(int value)
   {
      inphaseControlEffort.set(((double)((short) value)) / conversionFactor);// / 100.0);
   }
   
   public double getValue()
   {
	   return inphaseControlEffort.getDoubleValue();
   }

}
