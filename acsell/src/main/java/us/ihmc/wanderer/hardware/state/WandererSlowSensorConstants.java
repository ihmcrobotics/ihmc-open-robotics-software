package us.ihmc.wanderer.hardware.state;

import us.ihmc.acsell.hardware.state.slowSensors.AcsellSlowSensorConstants;

public class WandererSlowSensorConstants implements AcsellSlowSensorConstants
{

   @Override
   public double getBusVoltageConversion()
   {
      return 3.3 / 4096 * 100.0 / 2.2;
   }

   @Override
   public double getBoardThermistorBeta()
   {
      return 3730;
   }

   @Override
   public double getBoardThermistorRoomTempResistance()
   {
      return 22000;
   }

   @Override
   public double getBoardThermistorSeriesResistance()
   {
      return 100.0e3;
   }

   @Override
   public double getBoardThermistorSourceVoltage()
   {
      return 12.0;
   }

   @Override
   public double getBoardThermistorADCCounts()
   {
      return 4096.0;
   }

   @Override
   public double getBoardThermistorADCMaxVoltage()
   {
      return 3.3;
   }

   @Override
   public double getControlVoltageConversion()
   {
      return 100.0;
   }

   @Override
   public double getMotorTemperatureConversion()
   {
      return 100.0;
   }

   @Override
   public double getPressureSensorOffset()
   {
      return -63.5;//-13.0;//Changes with altitude: -63.5 in Albuquerque -76.5 near sea level.
   }

   @Override
   public double getPressureSensorScale()
   {
      return 0.0815 / 16.0;
   }

   @Override
   public double getPressureSensorConversion()
   {
      return 5.0 / 65536.0;
   }

   @Override
   public double getCurrentSensorConversion()
   {
      return -3.3 / 4096.0 / 40.0 / 0.0015;
   }

   @Override
   public double getStrainSensorConversion()
   {
      return 5.0 / 65535.0;
   }
   
}
