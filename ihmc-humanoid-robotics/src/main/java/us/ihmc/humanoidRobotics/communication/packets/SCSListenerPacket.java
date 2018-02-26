package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

/**
 * User: Matt Date: 3/26/13
 */
public class SCSListenerPacket extends Packet<SCSListenerPacket>
{
   public boolean isStopped = true;

   public SCSListenerPacket()
   {
   }

   @Override
   public void set(SCSListenerPacket other)
   {
      isStopped = other.isStopped;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SCSListenerPacket other, double epsilon)
   {
      return other.isStopped == isStopped;
   }
}
