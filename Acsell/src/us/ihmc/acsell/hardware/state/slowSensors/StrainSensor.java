package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StrainSensor implements AcsellSlowSensor
{
   private final DoubleYoVariable strainSensor;
   private double gain = 1, offset = 0;

   public StrainSensor(String name, int sensor, YoVariableRegistry registry)
   {
      strainSensor = new DoubleYoVariable(name + "StrainSensor" + sensor, registry);
   }

   @Override
   public void update(int value)
   {
      strainSensor.set(((double) value) * 5.0 / 65535.0);
   }

   public double getCalibratedValue()
   {
      return (strainSensor.getValueAsDouble() - offset) * gain; //see Spencer's email
   }

   public void setCalibration(double gain, double offset)
   {
      this.gain = gain;
      this.offset = offset;
   }

   public void tare()
   {
      this.offset = strainSensor.getValueAsDouble();
   }
}
