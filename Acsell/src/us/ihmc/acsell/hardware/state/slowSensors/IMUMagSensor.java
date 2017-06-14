package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.DoubleYoVariable;

public class IMUMagSensor implements AcsellSlowSensor
{
   private final DoubleYoVariable imuMag;
   
   public IMUMagSensor(String name, String axis, YoVariableRegistry registry)
   {
      imuMag = new DoubleYoVariable(name + "IMUMag" + axis, registry);
   }

   @Override
   public void update(int value)
   {
      imuMag.set(((short)value));
   }

   public double getValue()
   {
      return imuMag.getDoubleValue();
   }

}
