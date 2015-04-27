package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

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
