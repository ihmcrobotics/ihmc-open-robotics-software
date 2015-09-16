package us.ihmc.atlas.ros;

import java.net.URI;
import java.util.LinkedHashMap;
import java.util.concurrent.ArrayBlockingQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosBoolPublisher;
import us.ihmc.utilities.ros.publisher.RosCachedRawIMUDataPublisher;
import us.ihmc.utilities.ros.publisher.RosDoublePublisher;
import us.ihmc.utilities.ros.publisher.RosInt64Publisher;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosAtlasAuxiliaryRobotDataPublisher implements PacketConsumer<RobotConfigurationData>, Runnable
{
   private final ArrayBlockingQueue<AtlasAuxiliaryRobotData> availableAtlasAuxiliaryData = new ArrayBlockingQueue<>(30);
   private final RosMainNode rosMainNode;

   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosBoolPublisher> electricForearmEnabledPublishers = new LinkedHashMap<>();
   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosDoublePublisher> electricForearmTemperaturePublishers = new LinkedHashMap<>();
   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosDoublePublisher> electricForearmCurrentPublishers = new LinkedHashMap<>();

   private final RosDoublePublisher pumpInletPressurePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher pumpSupplyPressurePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher airSumpPressurePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher pumpSupplyTemperaturePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher pumpRPMPublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher motorTemperaturePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher motorDriverTemperaturePublisher = new RosDoublePublisher(false);

   private final RosBoolPublisher batteryChargingPublisher = new RosBoolPublisher(false);
   private final RosDoublePublisher batteryVoltagePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher batteryCurrentPublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher remainingBatteryTimePublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher remainingAmpHoursPublisher = new RosDoublePublisher(false);
   private final RosDoublePublisher remainingChargePercentagePublisher = new RosDoublePublisher(false);
   private final RosInt64Publisher cycleCountPublisher = new RosInt64Publisher(false);
   private final RosCachedRawIMUDataPublisher rawImuBatchPublisher = new RosCachedRawIMUDataPublisher(false, "/pelvis_imu");

   public RosAtlasAuxiliaryRobotDataPublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;
      initialize(rosMainNode, rosNameSpace);
   }

   public RosAtlasAuxiliaryRobotDataPublisher(URI rosuri, String nameSpace)
   {
      this.rosMainNode = new RosMainNode(rosuri, nameSpace + getClass().getName());
      initialize(rosMainNode, nameSpace);
   }
   
   private void initialize(RosMainNode rosMainNode, String rosNameSpace)
   {
      setupElectricForearmPublishers(rosNameSpace);
      setupPumpPublishers(rosNameSpace);
      setupBatteryPublishers(rosMainNode, rosNameSpace);
      setupBatchImuPublisher(rosMainNode, rosNameSpace);
      
      Thread thread = new Thread(this, getClass().getName());
      thread.start();
   }

   private void setupBatchImuPublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      rosMainNode.attachPublisher(rosNameSpace + "/output/imu/raw_imu_sensor_batch", rawImuBatchPublisher);
   }

   private void setupBatteryPublishers(RosMainNode rosMainNode, String rosNameSpace)
   {
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/charging", batteryChargingPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/voltage", batteryVoltagePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/current", batteryCurrentPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/time_remaining", remainingBatteryTimePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/amp_hours_remaining", remainingAmpHoursPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/charge_percentage_remaining", remainingChargePercentagePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/battery/cycle_count", cycleCountPublisher);
   }

   private void setupPumpPublishers(String rosNameSpace)
   {
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/pump_inlet_pressure", pumpInletPressurePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/pump_supply_pressure", pumpSupplyPressurePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/air_sump_pressure", airSumpPressurePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/pump_supply_temperature", pumpSupplyTemperaturePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/pump_rpm", pumpRPMPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/motor_temperature", motorTemperaturePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pump/motor_driver_temperature", motorDriverTemperaturePublisher);
   }

   private void setupElectricForearmPublishers(String rosNameSpace)
   {
      for (AtlasElectricMotorPacketEnum value : AtlasElectricMotorPacketEnum.values)
      {
         RosBoolPublisher boolPublisher = new RosBoolPublisher(false);
         electricForearmEnabledPublishers.put(value, boolPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/enabled", boolPublisher);

         RosDoublePublisher tempPublisher = new RosDoublePublisher(false);
         electricForearmTemperaturePublishers.put(value, tempPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/temperature", tempPublisher);

         RosDoublePublisher currentPublisher = new RosDoublePublisher(false);
         electricForearmCurrentPublishers.put(value, currentPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/drive_current", currentPublisher);
      }
   }

   @Override public void run()
   {
      while (true)
      {
         AtlasAuxiliaryRobotData auxiliaryRobotData;
         try
         {
            auxiliaryRobotData = availableAtlasAuxiliaryData.take();
         }
         catch (InterruptedException e)
         {
            // Continue on
            continue;
         }
         if(rosMainNode.isStarted())
         {
            publishElectricForearmData(auxiliaryRobotData);

            publishPumpData(auxiliaryRobotData);

            publishBatteryData(auxiliaryRobotData);
            
            publishImuData(auxiliaryRobotData);
         }
      }
   }

   private void publishImuData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      rawImuBatchPublisher.publish(auxiliaryRobotData.rawImuDeltas, auxiliaryRobotData.rawImuRates, auxiliaryRobotData.rawImuTimestamps,
            auxiliaryRobotData.rawImuPacketCounts);
   }

   private void publishElectricForearmData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      for (AtlasElectricMotorPacketEnum value : AtlasElectricMotorPacketEnum.values)
      {
         RosBoolPublisher rosBoolPublisher = electricForearmEnabledPublishers.get(value);
         rosBoolPublisher.publish(auxiliaryRobotData.electricJointEnabledArray[value.getId()]);

         RosDoublePublisher temperaturePublisher = electricForearmTemperaturePublishers.get(value);
         temperaturePublisher.publish(auxiliaryRobotData.electricJointTemperatures[value.getId()]);

         RosDoublePublisher currentPublisher = electricForearmCurrentPublishers.get(value);
         currentPublisher.publish(auxiliaryRobotData.electricJointCurrents[value.getId()]);
      }
   }

   private void publishPumpData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      pumpInletPressurePublisher.publish(auxiliaryRobotData.pumpInletPressure);
      pumpSupplyPressurePublisher.publish(auxiliaryRobotData.pumpSupplyPressure);
      airSumpPressurePublisher.publish(auxiliaryRobotData.airSumpPressure);
      pumpSupplyTemperaturePublisher.publish(auxiliaryRobotData.pumpSupplyTemperature);
      pumpRPMPublisher.publish(auxiliaryRobotData.pumpRPM);
      motorTemperaturePublisher.publish(auxiliaryRobotData.motorTemperature);
      motorDriverTemperaturePublisher.publish(auxiliaryRobotData.motorDriverTemperature);
   }

   private void publishBatteryData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      batteryChargingPublisher.publish(auxiliaryRobotData.batteryCharging);
      batteryVoltagePublisher.publish(auxiliaryRobotData.batteryVoltage);
      batteryCurrentPublisher.publish(auxiliaryRobotData.batteryCurrent);
      remainingBatteryTimePublisher.publish(auxiliaryRobotData.remainingBatteryTime);
      remainingAmpHoursPublisher.publish(auxiliaryRobotData.remainingAmpHours);
      remainingChargePercentagePublisher.publish(auxiliaryRobotData.remainingChargePercentage);
      cycleCountPublisher.publish(auxiliaryRobotData.batteryCycleCount);
   }

   @Override public void receivedPacket(RobotConfigurationData packet)
   {
      AuxiliaryRobotData auxiliaryRobotData = packet.getAuxiliaryRobotData();

      if(auxiliaryRobotData != null && auxiliaryRobotData instanceof AtlasAuxiliaryRobotData)
      {
         if(!availableAtlasAuxiliaryData.offer((AtlasAuxiliaryRobotData) auxiliaryRobotData))
         {
            availableAtlasAuxiliaryData.clear();
         }
      }
   }
}
