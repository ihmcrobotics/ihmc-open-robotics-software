package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class QuadratureControlEffort implements StepprSlowSensor
{
   private final DoubleYoVariable quadratureControlEffort;
   
   public QuadratureControlEffort(String name, YoVariableRegistry registry)
   {
      quadratureControlEffort = new DoubleYoVariable(name + "QuadratureControlEffort", registry);
   }

   @Override
   public void update(int value)
   {
      quadratureControlEffort.set(((double)((short) value)) / 100.0);
   }

}
