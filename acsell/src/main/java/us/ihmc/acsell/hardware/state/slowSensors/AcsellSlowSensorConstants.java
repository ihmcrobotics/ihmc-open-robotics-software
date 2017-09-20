package us.ihmc.acsell.hardware.state.slowSensors;

public interface AcsellSlowSensorConstants
{
   public abstract double getBusVoltageConversion();
   public abstract double getBoardThermistorBeta();
   public abstract double getBoardThermistorRoomTempResistance();
   public abstract double getBoardThermistorSeriesResistance();
   public abstract double getBoardThermistorSourceVoltage();
   public abstract double getBoardThermistorADCCounts();
   public abstract double getBoardThermistorADCMaxVoltage();
   public abstract double getControlVoltageConversion();
   public abstract double getMotorTemperatureConversion();
   public abstract double getPressureSensorOffset();
   public abstract double getPressureSensorScale();
   public abstract double getPressureSensorConversion();
   public abstract double getCurrentSensorConversion();
   public abstract double getStrainSensorConversion();
}
