package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.IntegerYoVariable;

public class StatorHallSwitches implements AcsellSlowSensor
{
   private final IntegerYoVariable statorHallSwitches;
   
   public StatorHallSwitches(String name, YoVariableRegistry registry)
   {
      statorHallSwitches = new IntegerYoVariable(name + "StatorHallSwitches", registry);
   }

   @Override
   public void update(int value)
   {
      statorHallSwitches.set(value);
   }

}
