package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.idl.RecyclingArrayListPubSub;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasAuxiliaryRobotData extends Packet<AtlasAuxiliaryRobotData>
{
   public TFloatArrayList electricJointTemperatures = new TFloatArrayList();
   public TFloatArrayList electricJointCurrents = new TFloatArrayList();
   public TByteArrayList electricJointEnabledArray = new TByteArrayList(); // TODO change back to an array of booleans

   public TLongArrayList rawImuTimestamps = new TLongArrayList();
   public TLongArrayList rawImuPacketCounts = new TLongArrayList();
   public RecyclingArrayListPubSub<Vector3D32> rawImuRates = new RecyclingArrayListPubSub<>(Vector3D32.class, Vector3D32::new, 1);
   public RecyclingArrayListPubSub<Vector3D32> rawImuDeltas = new RecyclingArrayListPubSub<>(Vector3D32.class, Vector3D32::new, 1);

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
   }

   public void clearRawImuData()
   {
      rawImuTimestamps.reset();
      rawImuPacketCounts.reset();
      rawImuRates.clear();
      rawImuDeltas.clear();
   }

   public void addRawImuData(long timestamp, long packetCount, float dax, float day, float daz, float ddx, float ddy, float ddz)
   {
      rawImuTimestamps.add(timestamp);
      rawImuPacketCounts.add(packetCount);

      rawImuRates.add().set(dax, day, daz);
      rawImuDeltas.add().set(ddx, ddy, ddz);
   }

   public void clearElectricJointStatus()
   {
      electricJointEnabledArray.reset();
      electricJointTemperatures.reset();
      electricJointCurrents.reset();
   }

   public void addElectricJointStatus(boolean enabled, float driveCurrent, float driveTemperature)
   {
      electricJointEnabledArray.add((byte) (enabled ? 1 : 0));
      electricJointTemperatures.add(driveTemperature);
      electricJointCurrents.add(driveCurrent);
   }

   @Override
   public void set(AtlasAuxiliaryRobotData other)
   {
      batteryCharging = other.batteryCharging;
      batteryVoltage = other.batteryVoltage;
      batteryCurrent = other.batteryCurrent;
      remainingBatteryTime = other.remainingBatteryTime;
      remainingAmpHours = other.remainingAmpHours;
      remainingChargePercentage = other.remainingChargePercentage;
      batteryCycleCount = other.batteryCycleCount;

      pumpInletPressure = other.pumpInletPressure;
      pumpSupplyPressure = other.pumpSupplyPressure;
      airSumpPressure = other.airSumpPressure;
      pumpSupplyTemperature = other.pumpSupplyTemperature;
      pumpRPM = other.pumpRPM;
      motorTemperature = other.motorTemperature;
      motorDriverTemperature = other.motorDriverTemperature;
      MessageTools.copyData(other.electricJointCurrents, electricJointCurrents);
      MessageTools.copyData(other.electricJointTemperatures, electricJointTemperatures);
      MessageTools.copyData(other.electricJointEnabledArray, electricJointEnabledArray);
      MessageTools.copyData(other.rawImuTimestamps, rawImuTimestamps);
      MessageTools.copyData(other.rawImuPacketCounts, rawImuPacketCounts);
      MessageTools.copyData(other.rawImuRates, rawImuRates);
      MessageTools.copyData(other.rawImuDeltas, rawImuDeltas);
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
      if (!MessageTools.epsilonEquals(electricJointTemperatures, other.electricJointTemperatures, (float) epsilon))
         return false;
      if (!MessageTools.epsilonEquals(electricJointCurrents, other.electricJointCurrents, (float) epsilon))
         return false;
      if (!electricJointEnabledArray.equals(other.electricJointEnabledArray))
         return false;
      if (!rawImuTimestamps.equals(other.rawImuTimestamps))
         return false;
      if (!rawImuPacketCounts.equals(other.rawImuPacketCounts))
         return false;

      if (!MessageTools.epsilonEquals(rawImuRates, other.rawImuRates, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(rawImuDeltas, other.rawImuDeltas, epsilon))
         return false;
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
