package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ZEDSVOCurrentFileMessage extends Packet<ZEDSVOCurrentFileMessage> implements Settable<ZEDSVOCurrentFileMessage>, EpsilonComparable<ZEDSVOCurrentFileMessage>
{
   public java.lang.StringBuilder current_file_name_;
   public byte record_mode_;
   public long current_position_;
   public long length_;

   public ZEDSVOCurrentFileMessage()
   {
      current_file_name_ = new java.lang.StringBuilder(255);
   }

   public ZEDSVOCurrentFileMessage(ZEDSVOCurrentFileMessage other)
   {
      this();
      set(other);
   }

   public void set(ZEDSVOCurrentFileMessage other)
   {
      current_file_name_.setLength(0);
      current_file_name_.append(other.current_file_name_);

      record_mode_ = other.record_mode_;

      current_position_ = other.current_position_;

      length_ = other.length_;

   }

   public void setCurrentFileName(java.lang.String current_file_name)
   {
      current_file_name_.setLength(0);
      current_file_name_.append(current_file_name);
   }

   public java.lang.String getCurrentFileNameAsString()
   {
      return getCurrentFileName().toString();
   }
   public java.lang.StringBuilder getCurrentFileName()
   {
      return current_file_name_;
   }

   public void setRecordMode(byte record_mode)
   {
      record_mode_ = record_mode;
   }
   public byte getRecordMode()
   {
      return record_mode_;
   }

   public void setCurrentPosition(long current_position)
   {
      current_position_ = current_position;
   }
   public long getCurrentPosition()
   {
      return current_position_;
   }

   public void setLength(long length)
   {
      length_ = length;
   }
   public long getLength()
   {
      return length_;
   }


   public static Supplier<ZEDSVOCurrentFileMessagePubSubType> getPubSubType()
   {
      return ZEDSVOCurrentFileMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ZEDSVOCurrentFileMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ZEDSVOCurrentFileMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.current_file_name_, other.current_file_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.record_mode_, other.record_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_, other.current_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_, other.length_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ZEDSVOCurrentFileMessage)) return false;

      ZEDSVOCurrentFileMessage otherMyClass = (ZEDSVOCurrentFileMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.current_file_name_, otherMyClass.current_file_name_)) return false;

      if(this.record_mode_ != otherMyClass.record_mode_) return false;

      if(this.current_position_ != otherMyClass.current_position_) return false;

      if(this.length_ != otherMyClass.length_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ZEDSVOCurrentFileMessage {");
      builder.append("current_file_name=");
      builder.append(this.current_file_name_);      builder.append(", ");
      builder.append("record_mode=");
      builder.append(this.record_mode_);      builder.append(", ");
      builder.append("current_position=");
      builder.append(this.current_position_);      builder.append(", ");
      builder.append("length=");
      builder.append(this.length_);
      builder.append("}");
      return builder.toString();
   }
}
