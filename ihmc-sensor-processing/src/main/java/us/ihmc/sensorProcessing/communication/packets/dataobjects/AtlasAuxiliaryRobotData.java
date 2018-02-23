package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.Arrays;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.ArrayTools;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasAuxiliaryRobotData extends Packet<AtlasAuxiliaryRobotData>
{
   public float[] electricJointTemperatures;
   public float[] electricJointCurrents;
   public boolean[] electricJointEnabledArray;

   public long[] rawImuTimestamps = new long[15];
   public long[] rawImuPacketCounts = new long[15];
   public float[][] rawImuRates = new float[15][3];
   public float[][] rawImuDeltas = new float[15][3];

   public boolean batteryCharging;
   public float batteryVoltage;
   public float batteryCurrent;
   public float remainingBatteryTime;
   public float remainingAmpHours;
   public float remainingChargePercentage;
   public long batteryCycleCount;

   public float pumpInletPressure;
   public float pumpSupplyPressure;
   public float airSumpPressure;
   public float pumpSupplyTemperature;
   public float pumpRPM;
   public float motorTemperature;
   public float motorDriverTemperature;

   public AtlasAuxiliaryRobotData()
   {
      electricJointTemperatures = new float[6];
      electricJointEnabledArray = new boolean[6];
      electricJointCurrents = new float[6];
   }

   public void updateRawImuData(int index, long timestamp, long packetCount, float dax, float day, float daz, float ddx, float ddy, float ddz)
   {
      rawImuTimestamps[index] = timestamp;
      rawImuPacketCounts[index] = packetCount;

      rawImuRates[index][0] = dax;
      rawImuRates[index][1] = day;
      rawImuRates[index][2] = daz;

      rawImuDeltas[index][0] = ddx;
      rawImuDeltas[index][1] = ddy;
      rawImuDeltas[index][2] = ddz;
   }

   public void updateElectricJointStatus(int index, boolean enabled, float driveCurrent, float driveTemperature)
   {
      electricJointEnabledArray[index] = enabled;
      electricJointTemperatures[index] = driveTemperature;
      electricJointCurrents[index] = driveCurrent;
   }

   @Override
   public void set(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      batteryCharging = auxiliaryRobotData.batteryCharging;
      batteryVoltage = auxiliaryRobotData.batteryVoltage;
      batteryCurrent = auxiliaryRobotData.batteryCurrent;
      remainingBatteryTime = auxiliaryRobotData.remainingBatteryTime;
      remainingAmpHours = auxiliaryRobotData.remainingAmpHours;
      remainingChargePercentage = auxiliaryRobotData.remainingChargePercentage;
      batteryCycleCount = auxiliaryRobotData.batteryCycleCount;

      pumpInletPressure = auxiliaryRobotData.pumpInletPressure;
      pumpSupplyPressure = auxiliaryRobotData.pumpSupplyPressure;
      airSumpPressure = auxiliaryRobotData.airSumpPressure;
      pumpSupplyTemperature = auxiliaryRobotData.pumpSupplyTemperature;
      pumpRPM = auxiliaryRobotData.pumpRPM;
      motorTemperature = auxiliaryRobotData.motorTemperature;
      motorDriverTemperature = auxiliaryRobotData.motorDriverTemperature;

      for (int i = 0; i < 6; i++)
      {
         electricJointCurrents[i] = auxiliaryRobotData.electricJointCurrents[i];
         electricJointTemperatures[i] = auxiliaryRobotData.electricJointTemperatures[i];
         electricJointEnabledArray[i] = auxiliaryRobotData.electricJointEnabledArray[i];
      }

      for (int i = 0; i < 15; i++)
      {
         rawImuTimestamps[i] = auxiliaryRobotData.rawImuTimestamps[i];
         rawImuPacketCounts[i] = auxiliaryRobotData.rawImuPacketCounts[i];

         for (int j = 0; j < 3; j++)
         {
            rawImuRates[i][j] = auxiliaryRobotData.rawImuRates[i][j];
            rawImuDeltas[i][j] = auxiliaryRobotData.rawImuDeltas[i][j];
         }
      }
   }

   public boolean isBatteryCharging()
   {
      return batteryCharging;
   }

   public void setBatteryCharging(boolean batteryCharging)
   {
      this.batteryCharging = batteryCharging;
   }

   public float getBatteryVoltage()
   {
      return batteryVoltage;
   }

   public void setBatteryVoltage(float batteryVoltage)
   {
      this.batteryVoltage = batteryVoltage;
   }

   public float getBatteryCurrent()
   {
      return batteryCurrent;
   }

   public void setBatteryCurrent(float batteryCurrent)
   {
      this.batteryCurrent = batteryCurrent;
   }

   public float getRemainingBatteryTime()
   {
      return remainingBatteryTime;
   }

   public void setRemainingBatteryTime(float remainingBatteryTime)
   {
      this.remainingBatteryTime = remainingBatteryTime;
   }

   public float getRemainingAmpHours()
   {
      return remainingAmpHours;
   }

   public void setRemainingAmpHours(float remainingAmpHours)
   {
      this.remainingAmpHours = remainingAmpHours;
   }

   public float getRemainingChargePercentage()
   {
      return remainingChargePercentage;
   }

   public void setRemainingChargePercentage(float remainingChargePercentage)
   {
      this.remainingChargePercentage = remainingChargePercentage;
   }

   public long getBatteryCycleCount()
   {
      return batteryCycleCount;
   }

   public void setBatteryCycleCount(long batteryCycleCount)
   {
      this.batteryCycleCount = batteryCycleCount;
   }

   public float getPumpInletPressure()
   {
      return pumpInletPressure;
   }

   public void setPumpInletPressure(float pumpInletPressure)
   {
      this.pumpInletPressure = pumpInletPressure;
   }

   public float getPumpSupplyPressure()
   {
      return pumpSupplyPressure;
   }

   public void setPumpSupplyPressure(float pumpSupplyPressure)
   {
      this.pumpSupplyPressure = pumpSupplyPressure;
   }

   public float getAirSumpPressure()
   {
      return airSumpPressure;
   }

   public void setAirSumpPressure(float airSumpPressure)
   {
      this.airSumpPressure = airSumpPressure;
   }

   public float getPumpSupplyTemperature()
   {
      return pumpSupplyTemperature;
   }

   public void setPumpSupplyTemperature(float pumpSupplyTemperature)
   {
      this.pumpSupplyTemperature = pumpSupplyTemperature;
   }

   public float getPumpRPM()
   {
      return pumpRPM;
   }

   public void setPumpRPM(float pumpRPM)
   {
      this.pumpRPM = pumpRPM;
   }

   public float getMotorTemperature()
   {
      return motorTemperature;
   }

   public void setMotorTemperature(float motorTemperature)
   {
      this.motorTemperature = motorTemperature;
   }

   public float getMotorDriverTemperature()
   {
      return motorDriverTemperature;
   }

   public void setMotorDriverTemperature(float motorDriverTemperature)
   {
      this.motorDriverTemperature = motorDriverTemperature;
   }

   @Override
   public boolean epsilonEquals(AtlasAuxiliaryRobotData other, double epsilon)
   {
      if (!ArrayTools.deltaEquals(electricJointTemperatures, other.electricJointTemperatures, (float) epsilon))
         return false;
      if (!ArrayTools.deltaEquals(electricJointCurrents, other.electricJointCurrents, (float) epsilon))
         return false;
      if (!Arrays.equals(electricJointEnabledArray, other.electricJointEnabledArray))
         return false;
      if (!Arrays.equals(rawImuTimestamps, other.rawImuTimestamps))
         return false;
      if (!Arrays.equals(rawImuPacketCounts, other.rawImuPacketCounts))
         return false;

      for (int i = 0; i < 15; i++)
      {
         if (!ArrayTools.deltaEquals(rawImuRates[i], other.rawImuRates[i], (float) epsilon))
            return false;
         if (!ArrayTools.deltaEquals(rawImuDeltas[i], other.rawImuDeltas[i], (float) epsilon))
            return false;
      }

      if (batteryCharging != other.batteryCharging)
         return false;
      if (!MathTools.epsilonEquals(batteryVoltage, other.batteryVoltage, epsilon))
         return false;
      if (!MathTools.epsilonEquals(batteryCurrent, other.batteryCurrent, epsilon))
         return false;
      if (!MathTools.epsilonEquals(remainingBatteryTime, other.remainingBatteryTime, epsilon))
         return false;
      if (!MathTools.epsilonEquals(remainingAmpHours, other.remainingAmpHours, epsilon))
         return false;
      if (!MathTools.epsilonEquals(remainingChargePercentage, other.remainingChargePercentage, epsilon))
         return false;
      if (!MathTools.epsilonEquals(batteryCycleCount, other.batteryCycleCount, epsilon))
         return false;
      if (!MathTools.epsilonEquals(pumpInletPressure, other.pumpInletPressure, epsilon))
         return false;
      if (!MathTools.epsilonEquals(pumpSupplyPressure, other.pumpSupplyPressure, epsilon))
         return false;
      if (!MathTools.epsilonEquals(airSumpPressure, other.airSumpPressure, epsilon))
         return false;
      if (!MathTools.epsilonEquals(pumpSupplyTemperature, other.pumpSupplyTemperature, epsilon))
         return false;
      if (!MathTools.epsilonEquals(pumpRPM, other.pumpRPM, epsilon))
         return false;
      if (!MathTools.epsilonEquals(motorTemperature, other.motorTemperature, epsilon))
         return false;
      if (!MathTools.epsilonEquals(motorDriverTemperature, other.motorDriverTemperature, epsilon))
         return false;
      return true;
   }
}
