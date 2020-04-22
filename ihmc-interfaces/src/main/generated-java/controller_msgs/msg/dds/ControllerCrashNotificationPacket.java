package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message notifying if the IHMC whole-body controller has crashed unexpectedly.
       */
public class ControllerCrashNotificationPacket extends Packet<ControllerCrashNotificationPacket> implements Settable<ControllerCrashNotificationPacket>, EpsilonComparable<ControllerCrashNotificationPacket>
{

   public static final byte CONTROLLER_READ = (byte) 0;

   public static final byte CONTROLLER_WRITE = (byte) 1;

   public static final byte CONTROLLER_RUN = (byte) 2;

   public static final byte ESTIMATOR_READ = (byte) 3;

   public static final byte ESTIMATOR_WRITE = (byte) 4;

   public static final byte ESTIMATOR_RUN = (byte) 5;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte controller_crash_location_ = (byte) 255;

   public java.lang.StringBuilder exception_type_;

   public java.lang.StringBuilder error_message_;

   public us.ihmc.idl.IDLSequence.StringBuilderHolder  stacktrace_;

   public ControllerCrashNotificationPacket()
   {



      exception_type_ = new java.lang.StringBuilder(255);

      error_message_ = new java.lang.StringBuilder(255);

      stacktrace_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (50, "type_d");

   }

   public ControllerCrashNotificationPacket(ControllerCrashNotificationPacket other)
   {
      this();
      set(other);
   }

   public void set(ControllerCrashNotificationPacket other)
   {

      sequence_id_ = other.sequence_id_;


      controller_crash_location_ = other.controller_crash_location_;


      exception_type_.setLength(0);
      exception_type_.append(other.exception_type_);


      error_message_.setLength(0);
      error_message_.append(other.error_message_);


      stacktrace_.set(other.stacktrace_);
   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setControllerCrashLocation(byte controller_crash_location)
   {
      controller_crash_location_ = controller_crash_location;
   }
   public byte getControllerCrashLocation()
   {
      return controller_crash_location_;
   }


   public void setExceptionType(java.lang.String exception_type)
   {
      exception_type_.setLength(0);
      exception_type_.append(exception_type);
   }

   public java.lang.String getExceptionTypeAsString()
   {
      return getExceptionType().toString();
   }
   public java.lang.StringBuilder getExceptionType()
   {
      return exception_type_;
   }


   public void setErrorMessage(java.lang.String error_message)
   {
      error_message_.setLength(0);
      error_message_.append(error_message);
   }

   public java.lang.String getErrorMessageAsString()
   {
      return getErrorMessage().toString();
   }
   public java.lang.StringBuilder getErrorMessage()
   {
      return error_message_;
   }



   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getStacktrace()
   {
      return stacktrace_;
   }


   public static Supplier<ControllerCrashNotificationPacketPubSubType> getPubSubType()
   {
      return ControllerCrashNotificationPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerCrashNotificationPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerCrashNotificationPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.controller_crash_location_, other.controller_crash_location_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.exception_type_, other.exception_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.error_message_, other.error_message_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.stacktrace_, other.stacktrace_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerCrashNotificationPacket)) return false;

      ControllerCrashNotificationPacket otherMyClass = (ControllerCrashNotificationPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.controller_crash_location_ != otherMyClass.controller_crash_location_) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.exception_type_, otherMyClass.exception_type_)) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.error_message_, otherMyClass.error_message_)) return false;


      if (!this.stacktrace_.equals(otherMyClass.stacktrace_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerCrashNotificationPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("controller_crash_location=");
      builder.append(this.controller_crash_location_);      builder.append(", ");

      builder.append("exception_type=");
      builder.append(this.exception_type_);      builder.append(", ");

      builder.append("error_message=");
      builder.append(this.error_message_);      builder.append(", ");

      builder.append("stacktrace=");
      builder.append(this.stacktrace_);
      builder.append("}");
      return builder.toString();
   }
}
