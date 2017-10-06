package us.ihmc.communication.packets;

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
