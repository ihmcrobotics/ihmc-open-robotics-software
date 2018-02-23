package us.ihmc.sensorProcessing.communication.packets.dataobjects;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasAuxiliaryRobotData implements AuxiliaryRobotData
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

   @Override public void setAuxiliaryRobotData(AuxiliaryRobotData auxiliaryRobotData)
   {
      if(!(auxiliaryRobotData instanceof AtlasAuxiliaryRobotData))
      {
         throw new RuntimeException("Mismatches in Auxiliary Robot Data, cannot proceed");
      }
      else
      {
         this.batteryCharging = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).batteryCharging;
         this.batteryVoltage = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).batteryVoltage;
         this.batteryCurrent = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).batteryCurrent;
         this.remainingBatteryTime = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).remainingBatteryTime;
         this.remainingAmpHours = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).remainingAmpHours;
         this.remainingChargePercentage = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).remainingChargePercentage;
         this.batteryCycleCount = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).batteryCycleCount;

         this.pumpInletPressure = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).pumpInletPressure;
         this.pumpSupplyPressure = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).pumpSupplyPressure;
         this.airSumpPressure = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).airSumpPressure;
         this.pumpSupplyTemperature = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).pumpSupplyTemperature;
         this.pumpRPM = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).pumpRPM;
         this.motorTemperature = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).motorTemperature;
         this.motorDriverTemperature = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).motorDriverTemperature;

         for(int i = 0; i < 6; i++)
         {
            this.electricJointCurrents[i] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).electricJointCurrents[i];
            this.electricJointTemperatures[i] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).electricJointTemperatures[i];
            this.electricJointEnabledArray[i] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).electricJointEnabledArray[i];
         }

         for(int i = 0; i < 15; i++)
         {
            this.rawImuTimestamps[i] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).rawImuTimestamps[i];
            this.rawImuPacketCounts[i] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).rawImuPacketCounts[i];

            for(int j = 0; j < 3; j++)
            {
               this.rawImuRates[i][j] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).rawImuRates[i][j];
               this.rawImuDeltas[i][j] = ((AtlasAuxiliaryRobotData) auxiliaryRobotData).rawImuDeltas[i][j];
            }
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
}
