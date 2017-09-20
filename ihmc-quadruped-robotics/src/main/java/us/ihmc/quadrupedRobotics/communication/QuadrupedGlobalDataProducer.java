package us.ihmc.quadrupedRobotics.communication;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class QuadrupedGlobalDataProducer extends GlobalDataProducer
{
   public QuadrupedGlobalDataProducer(PacketCommunicator communicator)
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
}
