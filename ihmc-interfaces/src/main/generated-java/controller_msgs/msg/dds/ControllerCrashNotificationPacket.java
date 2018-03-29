package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message notifying if the IHMC whole-body controller has crashed unexpectedly.
 */
public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket>
      implements Settable<ControllerCrashNotificationPacket>, EpsilonComparable<ControllerCrashNotificationPacket>
{
   public static final byte CONTROLLER_READ = (byte) 0;
   public static final byte CONTROLLER_WRITE = (byte) 1;
   public static final byte CONTROLLER_RUN = (byte) 2;
   public static final byte ESTIMATOR_READ = (byte) 3;
   public static final byte ESTIMATOR_WRITE = (byte) 4;
   public static final byte ESTIMATOR_RUN = (byte) 5;
   public byte controller_crash_location_ = (byte) 255;
   public java.lang.StringBuilder stacktrace_;

   public ControllerCrashNotificationPacket()
   {

      stacktrace_ = new java.lang.StringBuilder(255);
   }

   public ControllerCrashNotificationPacket(ControllerCrashNotificationPacket other)
   {
      set(other);
   }

   public void set(ControllerCrashNotificationPacket other)
   {
      controller_crash_location_ = other.controller_crash_location_;

      stacktrace_.setLength(0);
      stacktrace_.append(other.stacktrace_);
   }

   public byte getControllerCrashLocation()
   {
      return controller_crash_location_;
   }

   public void setControllerCrashLocation(byte controller_crash_location)
   {
      controller_crash_location_ = controller_crash_location;
   }

   public java.lang.String getStacktraceAsString()
   {
      return getStacktrace().toString();
   }

   public java.lang.StringBuilder getStacktrace()
   {
      return stacktrace_;
   }

   public void setStacktrace(java.lang.String stacktrace)
   {
      stacktrace_.setLength(0);
      stacktrace_.append(stacktrace);
   }

   @Override
   public boolean epsilonEquals(ControllerCrashNotificationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.controller_crash_location_, other.controller_crash_location_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.stacktrace_, other.stacktrace_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof ControllerCrashNotificationPacket))
         return false;

      ControllerCrashNotificationPacket otherMyClass = (ControllerCrashNotificationPacket) other;

      if (this.controller_crash_location_ != otherMyClass.controller_crash_location_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.stacktrace_, otherMyClass.stacktrace_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerCrashNotificationPacket {");
      builder.append("controller_crash_location=");
      builder.append(this.controller_crash_location_);

      builder.append(", ");
      builder.append("stacktrace=");
      builder.append(this.stacktrace_);

      builder.append("}");
      return builder.toString();
   }
}
