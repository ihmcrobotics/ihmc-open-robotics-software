package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class ControlMode implements StepprSlowSensor
{
   private final IntegerYoVariable controlMode;
   
   public ControlMode(String name, YoVariableRegistry registry)
   {
      controlMode = new IntegerYoVariable(name + "ControlMode", registry);
   }

   @Override
   public void update(int value)
   {
      controlMode.set(value);
   }
   
}
