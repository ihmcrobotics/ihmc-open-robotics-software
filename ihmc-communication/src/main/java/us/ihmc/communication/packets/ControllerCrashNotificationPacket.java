package us.ihmc.communication.packets;

public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket>
{
   public enum CrashLocation
   {
      CONTROLLER_READ,
      CONTROLLER_WRITE,
      CONTROLLER_RUN, 
      ESTIMATOR_READ,
      ESTIMATOR_WRITE,
      ESTIMATOR_RUN
   }
   
   
   public CrashLocation location;
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
