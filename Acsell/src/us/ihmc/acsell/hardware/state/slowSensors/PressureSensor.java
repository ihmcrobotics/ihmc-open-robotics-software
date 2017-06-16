package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PressureSensor implements AcsellSlowSensor
{
   private double offset;// = -11.5-58;//1.96;
   //private static final double scale = 0.0815;
   private final double scale;// = 0.0815;
   private final double conversionFactor;
   
   private double rawValue;
   private final YoDouble pressureSensorRawVoltage;
   private final YoDouble force;
   
   public PressureSensor(String name, int sensor, AcsellSlowSensorConstants slowSensorConstants, YoVariableRegistry registry, double offset)
   {
      this(name, sensor, slowSensorConstants, registry);
      this.offset=offset;
   }

   public PressureSensor(String name, int sensor, AcsellSlowSensorConstants slowSensorConstants, YoVariableRegistry registry)
   {
      this.offset = slowSensorConstants.getPressureSensorOffset();
      this.scale = slowSensorConstants.getPressureSensorScale();
      this.conversionFactor = slowSensorConstants.getPressureSensorConversion();
      pressureSensorRawVoltage = new YoDouble(name + "PressureSensorRawVoltage" + sensor, registry);
      force = new YoDouble(name + "Force" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      rawValue = value;
      pressureSensorRawVoltage.set(((double) value) * conversionFactor); // 5.0 / 4095.0);
      force.set(((double) value) * scale + offset);

   }

   public double getValue()
   {
      return force.getDoubleValue();
   }
   
   public double getRawValue()
   {
      return rawValue;
   }

  public void tare()
  {
     offset = -force.getValueAsDouble()+offset;
  }

}
