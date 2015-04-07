package us.ihmc.atlas.ros;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosBoolPublisher;
import us.ihmc.utilities.ros.publisher.RosDoublePublisher;

import java.util.LinkedHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosAtlasAuxiliaryRobotDataPublisher implements PacketConsumer<RobotConfigurationData>, Runnable
{
   private final ConcurrentLinkedQueue<AtlasAuxiliaryRobotData> availableAtlasAuxiliaryData = new ConcurrentLinkedQueue<>();
   private final RosMainNode rosMainNode;

   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosBoolPublisher> electricForearmEnabledPublishers = new LinkedHashMap<>();
   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosDoublePublisher> electricForearmTemperaturePublishers = new LinkedHashMap<>();
   private final LinkedHashMap<AtlasElectricMotorPacketEnum, RosDoublePublisher> electricForearmCurrentPublishers = new LinkedHashMap<>();

   public RosAtlasAuxiliaryRobotDataPublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;

      boolean latched = false;

      for (AtlasElectricMotorPacketEnum value : AtlasElectricMotorPacketEnum.values)
      {
         RosBoolPublisher boolPublisher = new RosBoolPublisher(latched);
         electricForearmEnabledPublishers.put(value, boolPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/enabled", boolPublisher);

         RosDoublePublisher tempPublisher = new RosDoublePublisher(latched);
         electricForearmTemperaturePublishers.put(value, tempPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/temperature", tempPublisher);

         RosDoublePublisher currentPublisher = new RosDoublePublisher(latched);
         electricForearmCurrentPublishers.put(value, currentPublisher);
         rosMainNode.attachPublisher(rosNameSpace + "/output/electric_forearms/" + value.toString().toLowerCase() + "/drive_current", currentPublisher);
      }

      Thread thread = new Thread(this, "RosAtlasAuxiliaryRobotDataPublisher");
      thread.start();
   }

   @Override public void run()
   {
      while (true)
      {
         AtlasAuxiliaryRobotData auxiliaryRobotData = availableAtlasAuxiliaryData.poll();
         if(auxiliaryRobotData != null)
         {
            if(rosMainNode.isStarted())
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
         }

         Thread.yield();
      }
   }

   @Override public void receivedPacket(RobotConfigurationData packet)
   {
      AuxiliaryRobotData auxiliaryRobotData = packet.getAuxiliaryRobotData();

      if(auxiliaryRobotData != null && auxiliaryRobotData instanceof AtlasAuxiliaryRobotData)
      {
         availableAtlasAuxiliaryData.add((AtlasAuxiliaryRobotData) auxiliaryRobotData);
         if(availableAtlasAuxiliaryData.size() > 30)
         {
            availableAtlasAuxiliaryData.clear();
         }
      }
   }
}
