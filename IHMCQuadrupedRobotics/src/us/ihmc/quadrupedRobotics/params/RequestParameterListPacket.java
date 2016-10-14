package us.ihmc.quadrupedRobotics.params;

import us.ihmc.communication.packets.Packet;

public class RequestParameterListPacket extends Packet<RequestParameterListPacket>
{
   public RequestParameterListPacket() // no-arg for serialization
   {
   }

   @Override
   public boolean epsilonEquals(RequestParameterListPacket other, double epsilon)
   {
      return true;
   }
}
