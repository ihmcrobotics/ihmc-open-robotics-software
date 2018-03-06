package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message notifies the user that a previously sent message was rejected by the whole-body controller.
 */
public class InvalidPacketNotificationPacket implements Settable<InvalidPacketNotificationPacket>, EpsilonComparable<InvalidPacketNotificationPacket>
{
   private java.lang.StringBuilder packet_class_simple_name_;
   private java.lang.StringBuilder error_message_;

   public InvalidPacketNotificationPacket()
   {
      packet_class_simple_name_ = new java.lang.StringBuilder(255);

      error_message_ = new java.lang.StringBuilder(255);
   }

   public InvalidPacketNotificationPacket(InvalidPacketNotificationPacket other)
   {
      set(other);
   }

   public void set(InvalidPacketNotificationPacket other)
   {
      packet_class_simple_name_.setLength(0);
      packet_class_simple_name_.append(other.packet_class_simple_name_);

      error_message_.setLength(0);
      error_message_.append(other.error_message_);
   }

   public java.lang.String getPacketClassSimpleNameAsString()
   {
      return getPacketClassSimpleName().toString();
   }

   public java.lang.StringBuilder getPacketClassSimpleName()
   {
      return packet_class_simple_name_;
   }

   public void setPacketClassSimpleName(String packet_class_simple_name)
   {
      packet_class_simple_name_.setLength(0);
      packet_class_simple_name_.append(packet_class_simple_name);
   }

   public java.lang.String getErrorMessageAsString()
   {
      return getErrorMessage().toString();
   }

   public java.lang.StringBuilder getErrorMessage()
   {
      return error_message_;
   }

   public void setErrorMessage(String error_message)
   {
      error_message_.setLength(0);
      error_message_.append(error_message);
   }

   @Override
   public boolean epsilonEquals(InvalidPacketNotificationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.packet_class_simple_name_, other.packet_class_simple_name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.error_message_, other.error_message_, epsilon))
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
      if (!(other instanceof InvalidPacketNotificationPacket))
         return false;

      InvalidPacketNotificationPacket otherMyClass = (InvalidPacketNotificationPacket) other;

      if (!us.ihmc.idl.IDLTools.equals(this.packet_class_simple_name_, otherMyClass.packet_class_simple_name_))
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.error_message_, otherMyClass.error_message_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("InvalidPacketNotificationPacket {");
      builder.append("packet_class_simple_name=");
      builder.append(this.packet_class_simple_name_);

      builder.append(", ");
      builder.append("error_message=");
      builder.append(this.error_message_);

      builder.append("}");
      return builder.toString();
   }
}