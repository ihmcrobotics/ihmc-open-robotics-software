package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ControllerTemperature2 implements StepprSlowSensor
{
   private final DoubleYoVariable mcbTemperature2;
   
   public ControllerTemperature2(String name, YoVariableRegistry parentRegistry)
   {
      mcbTemperature2 = new DoubleYoVariable(name + "MCBTemperature2", parentRegistry);
   }

   @Override
   public void update(int value)
   {
      double therm_div_volt = value * 3.3f / 4096.0f;
      double therm_r_meas = 10.0e3f / (3.3f / therm_div_volt - 1);
      double THERM_B = 4288;
      double THERM_R_25C = 50000;
      double motor_temp_c = 0;
      if (therm_div_volt > 0)
         motor_temp_c = THERM_B / Math.log(therm_r_meas / (THERM_R_25C * Math.exp(-THERM_B / 298.15))) - 273.15;

      mcbTemperature2.set(motor_temp_c);
   }
   
}
