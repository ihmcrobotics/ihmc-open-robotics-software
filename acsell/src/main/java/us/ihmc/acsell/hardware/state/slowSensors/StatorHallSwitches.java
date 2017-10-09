package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class StatorHallSwitches implements AcsellSlowSensor
{
   private final YoInteger statorHallSwitches;
   
   public StatorHallSwitches(String name, YoVariableRegistry registry)
   {
      statorHallSwitches = new YoInteger(name + "StatorHallSwitches", registry);
   }

   @Override
   public void update(int value)
   {
      statorHallSwitches.set(value);
   }

}
