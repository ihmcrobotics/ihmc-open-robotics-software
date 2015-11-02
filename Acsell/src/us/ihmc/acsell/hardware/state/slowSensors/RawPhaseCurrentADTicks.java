package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class RawPhaseCurrentADTicks implements AcsellSlowSensor
{
   private final IntegerYoVariable rawPhaseCurrentADTicks;
   private final DoubleYoVariable phaseCurrent;
   private final double conversionFactor;

   public RawPhaseCurrentADTicks(String name, String phase, double conversionFactor, YoVariableRegistry registry)
   {
      this.conversionFactor = conversionFactor;
      rawPhaseCurrentADTicks = new IntegerYoVariable(name + "RawPhase" + phase + "CurrentADTicks", registry);
      phaseCurrent = new DoubleYoVariable(name + "Phase" + phase + "Current", registry);
   }

   @Override
   public void update(int value)
   {
      rawPhaseCurrentADTicks.set(value);
      phaseCurrent.set(((double) value) * conversionFactor); //-3.3 / 4096.0 / 40.0 / 0.0015);
   }

}
