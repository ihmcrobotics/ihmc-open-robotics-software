package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class PressureSensor implements StepprSlowSensor
{
   private static final double offset = 1.0;
   private static final double scale = 13.0 * 9.81;
   
   private final DoubleYoVariable pressureSensorRawVoltage;
   private final DoubleYoVariable force;
   
   public PressureSensor(String name, int sensor, YoVariableRegistry registry)
   {
      pressureSensorRawVoltage = new DoubleYoVariable(name + "PressureSensorRawVoltage" + sensor, registry);
      force = new DoubleYoVariable(name + "Force" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      pressureSensorRawVoltage.set(((double) value) * 5.0/4095.0);
      
      force.set(Math.max(0, pressureSensorRawVoltage.getDoubleValue() - offset) * scale);
      
      
   }

   public double getValue()
   {
      return force.getDoubleValue();
   }

}
