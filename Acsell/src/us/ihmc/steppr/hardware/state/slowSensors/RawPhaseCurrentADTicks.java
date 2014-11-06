package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class RawPhaseCurrentADTicks implements StepprSlowSensor
{
   private final IntegerYoVariable rawPhaseCurrentADTicks;
   
   public RawPhaseCurrentADTicks(String name, String phase, YoVariableRegistry registry)
   {
      rawPhaseCurrentADTicks = new IntegerYoVariable(name + "RawPhase" + phase + "CurrentADTicks", registry);
   }

   @Override
   public void update(int value)
   {
      rawPhaseCurrentADTicks.set(value);
   }

}
