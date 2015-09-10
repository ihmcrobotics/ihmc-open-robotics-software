package us.ihmc.acsell.hardware.state.slowSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ControllerTemperature2 implements AcsellSlowSensor
{
   private final DoubleYoVariable mcbTemperature2;
   private final double THERM_B;// = 3730;
   private final double THERM_R_25C;// = 22000;
   private final double SERIES_RESISTANCE;// = 100.0e3;
   private final double V_S;// = 12.0;
   private final double MAX_ADC_COUNT;// = 4096.0;
   private final double MAX_ADC_VOLTAGE;// = 3.3;
   
   public ControllerTemperature2(String name, AcsellSlowSensorConstants slowSensorConstants, YoVariableRegistry parentRegistry)
   {
      mcbTemperature2 = new DoubleYoVariable(name + "MCBTemperature2", parentRegistry);
      this.THERM_B = slowSensorConstants.getBoardThermistorBeta();
      this.THERM_R_25C = slowSensorConstants.getBoardThermistorRoomTempResistance();
      this.SERIES_RESISTANCE = slowSensorConstants.getBoardThermistorSeriesResistance();
      this.V_S = slowSensorConstants.getBoardThermistorSourceVoltage();
      this.MAX_ADC_COUNT = slowSensorConstants.getBoardThermistorADCCounts();
      this.MAX_ADC_VOLTAGE = slowSensorConstants.getBoardThermistorADCMaxVoltage();
   }

   @Override
   public void update(int value)
   {
	  double motor_temp_c = 0;
	  if (value > 0)
	  {
		  double therm_r_meas = SERIES_RESISTANCE * value / (MAX_ADC_COUNT * V_S / MAX_ADC_VOLTAGE - value);
		  double therm_k = THERM_R_25C*Math.exp(-THERM_B/298.15);
		  motor_temp_c = THERM_B / Math.log(therm_r_meas / therm_k) - 273.15;
	  }
	  mcbTemperature2.set(motor_temp_c);
   }
   
}
