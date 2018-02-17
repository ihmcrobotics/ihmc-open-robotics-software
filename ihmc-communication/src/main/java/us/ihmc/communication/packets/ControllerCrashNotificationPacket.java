package us.ihmc.communication.packets;

public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket>
{
   public byte controllerCrashLocation;
   public String stacktrace;
   
   public ControllerCrashNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }

   @Override
   public void set(ControllerCrashNotificationPacket other)
   {
      controllerCrashLocation = other.controllerCrashLocation;
      stacktrace = other.stacktrace;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(ControllerCrashNotificationPacket other, double epsilon)
   {
      return true;
   }
}
