package us.ihmc.communication.packets;

public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket>
{
   public static final byte CONTROLLER_READ = 0;
   public static final byte CONTROLLER_WRITE = 1;
   public static final byte CONTROLLER_RUN = 2;
   public static final byte ESTIMATOR_READ = 3;
   public static final byte ESTIMATOR_WRITE = 4;
   public static final byte ESTIMATOR_RUN = 5;

   public byte controllerCrashLocation;
   public StringBuilder stacktrace = new StringBuilder();
   
   public ControllerCrashNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }

   @Override
   public void set(ControllerCrashNotificationPacket other)
   {
      controllerCrashLocation = other.controllerCrashLocation;
      stacktrace.setLength(0);
      stacktrace.append(other.stacktrace);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(ControllerCrashNotificationPacket other, double epsilon)
   {
      return true;
   }
}
