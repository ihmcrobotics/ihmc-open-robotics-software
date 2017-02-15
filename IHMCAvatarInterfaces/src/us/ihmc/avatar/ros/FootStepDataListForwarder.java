package us.ihmc.avatar.ros;

import java.util.ArrayList;
import java.util.List;

import ihmc_msgs.FootstepDataListRosMessage;
import ihmc_msgs.FootstepDataRosMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class FootStepDataListForwarder extends AbstractRosTopicSubscriber<ihmc_msgs.FootstepDataListRosMessage>
{
   private final PacketCommunicator controllerCommunicator;
   
   public FootStepDataListForwarder(PacketCommunicator controllerCommunicator)
   {
      super(ihmc_msgs.FootstepDataListRosMessage._TYPE);
      this.controllerCommunicator = controllerCommunicator;
   }

   @Override
   public void onNewMessage(FootstepDataListRosMessage message)
   {
      ArrayList<us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage> footStepDataArrayList = new ArrayList<us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage>();
      List<FootstepDataRosMessage> footStepDataList = message.getFootstepDataList();
      FootStepDataMessageConverter.convertFootStepDataList(footStepDataList, footStepDataArrayList);
      ExecutionMode executionMode = ExecutionMode.values[(int) message.getExecutionMode()];
      
      controllerCommunicator.send(new us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage(footStepDataArrayList,message.getDefaultSwingTime(),message.getDefaultTransferTime(),executionMode));
   }
}
