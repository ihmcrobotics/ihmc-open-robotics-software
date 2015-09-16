package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;
import java.util.List;

import ihmc_msgs.FootstepDataListMessage;
import ihmc_msgs.FootstepDataMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class FootStepDataListForwarder extends AbstractRosTopicSubscriber<ihmc_msgs.FootstepDataListMessage>
{
   private final PacketCommunicator controllerCommunicator;
   
   public FootStepDataListForwarder(PacketCommunicator controllerCommunicator)
   {
      super(ihmc_msgs.FootstepDataListMessage._TYPE);
      this.controllerCommunicator = controllerCommunicator;
   }

   @Override
   public void onNewMessage(FootstepDataListMessage message)
   {
      ArrayList<FootstepData> footStepDataArrayList = new ArrayList<FootstepData>();
      List<FootstepDataMessage> footStepDataList = message.getFootstepDataList();
      FootStepDataMessageConverter.convertFootStepDataList(footStepDataList, footStepDataArrayList);;
      
      controllerCommunicator.send(new FootstepDataList(footStepDataArrayList,message.getSwingTime(),message.getTransferTime()));
   }
}
