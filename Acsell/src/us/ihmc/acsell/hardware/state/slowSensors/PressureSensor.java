package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class PressureSensor implements AcsellSlowSensor
{
   private double offset = -11.5-58;//1.96;
   private static final double scale = 0.0815;

   private final DoubleYoVariable pressureSensorRawVoltage;
   private final DoubleYoVariable force;
   public PressureSensor(String name, int sensor, YoVariableRegistry registry, double offset)
   {
      this(name, sensor, registry);
      this.offset=offset;
   }

   public PressureSensor(String name, int sensor, YoVariableRegistry registry)
   {
      pressureSensorRawVoltage = new DoubleYoVariable(name + "PressureSensorRawVoltage" + sensor, registry);
      force = new DoubleYoVariable(name + "Force" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      pressureSensorRawVoltage.set(((double) value) * 5.0 / 4095.0);
      force.set(((double) value) * scale + offset);

   }

   public double getValue()
   {
      return force.getDoubleValue();
   }

  public void tare()
  {
     offset = -force.getValueAsDouble()+offset;
  }

}
