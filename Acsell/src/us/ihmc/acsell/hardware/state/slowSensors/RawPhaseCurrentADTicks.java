package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RawPhaseCurrentADTicks implements AcsellSlowSensor
{
   private final YoInteger rawPhaseCurrentADTicks;
   private final YoDouble phaseCurrent;
   private final double conversionFactor;

   public RawPhaseCurrentADTicks(String name, String phase, double conversionFactor, YoVariableRegistry registry)
   {
      this.conversionFactor = conversionFactor;
      rawPhaseCurrentADTicks = new YoInteger(name + "RawPhase" + phase + "CurrentADTicks", registry);
      phaseCurrent = new YoDouble(name + "Phase" + phase + "Current", registry);
   }

   @Override
   public void update(int value)
   {
      rawPhaseCurrentADTicks.set(value);
      phaseCurrent.set(((double) value) * conversionFactor); //-3.3 / 4096.0 / 40.0 / 0.0015);
   }

}
