package us.ihmc.avatar.ros.subscriber;

import std_msgs.Empty;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorCommandPacket;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;


public class RequestControllerStopSubscriber extends AbstractRosTopicSubscriber<std_msgs.Empty>
{
   private final PacketCommunicator packetCommunicator;
   
   public RequestControllerStopSubscriber(PacketCommunicator packetCommunicator)
   {
      super(std_msgs.Empty._TYPE);
      
      this.packetCommunicator = packetCommunicator;
   }

   @Override
   public void onNewMessage(Empty message)
   {
      BDIBehaviorCommandPacket packet = new BDIBehaviorCommandPacket(true);
      packet.setDestination(PacketDestination.CONTROLLER.ordinal());
      packetCommunicator.send(packet);
   }

}
