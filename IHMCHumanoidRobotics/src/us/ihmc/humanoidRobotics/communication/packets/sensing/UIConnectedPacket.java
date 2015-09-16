package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class UIConnectedPacket extends Packet<UIConnectedPacket>
{

   @Override
   public boolean epsilonEquals(UIConnectedPacket other, double epsilon)
   {
      return true;
   }

}
