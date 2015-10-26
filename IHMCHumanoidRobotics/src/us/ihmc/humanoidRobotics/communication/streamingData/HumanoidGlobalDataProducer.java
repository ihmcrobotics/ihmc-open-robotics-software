package us.ihmc.humanoidRobotics.communication.streamingData;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class HumanoidGlobalDataProducer extends GlobalDataProducer
{
   public HumanoidGlobalDataProducer(PacketCommunicator communicator)
   {
      super(communicator);
   }

   /**
    * Special method to directly send RobotConfigurationData, skipping the queue
    * @param robotConfigData
    */
   public void send(RobotConfigurationData robotConfigData)
   {
      communicator.send(robotConfigData);
   }

   /**
    * Special method to directly send HandJointAnglePacket, skipping the queue
    * @param handJointAnglePacket
    */
   public void send(HandJointAnglePacket handJointAnglePacket)
   {
      communicator.send(handJointAnglePacket);
   }


   /**
    * Special method to directly send CapturabilityBasedStatus, skipping the queue
    *
    * @param status
    */
   public void send(CapturabilityBasedStatus status)
   {
      communicator.send(status);
   }
}
