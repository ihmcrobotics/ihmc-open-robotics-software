package us.ihmc.quadrupedRobotics.params;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class RequestParameterListPacket extends Packet<RequestParameterListPacket>
{
   public RequestParameterListPacket()
   {
   }

   @Override
   public boolean epsilonEquals(RequestParameterListPacket other, double epsilon)
   {
      return false;
   }
}
