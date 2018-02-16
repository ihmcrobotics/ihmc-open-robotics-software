package us.ihmc.communication.packets;

public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket>
{
   public byte location;
   public String stacktrace;
   
   public ControllerCrashNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }

   @Override
   public void set(ControllerCrashNotificationPacket other)
   {
      location = other.location;
      stacktrace = other.stacktrace;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(ControllerCrashNotificationPacket other, double epsilon)
   {
      return true;
   }
}
