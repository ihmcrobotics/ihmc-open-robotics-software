package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class StatorHalSwitches implements AcsellSlowSensor
{
   private final IntegerYoVariable statorHallSwitches;
   
   public StatorHalSwitches(String name, YoVariableRegistry registry)
   {
      statorHallSwitches = new IntegerYoVariable(name + "StatorHallSwitches", registry);
   }

   @Override
   public void update(int value)
   {
      statorHallSwitches.set(value);
   }

}
