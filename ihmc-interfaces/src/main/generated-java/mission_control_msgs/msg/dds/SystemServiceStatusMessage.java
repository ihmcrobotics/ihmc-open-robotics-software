package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemServiceStatusMessage extends Packet<SystemServiceStatusMessage> implements Settable<SystemServiceStatusMessage>, EpsilonComparable<SystemServiceStatusMessage>
{
   public java.lang.StringBuilder service_name_;
   public java.lang.StringBuilder status_;
   public int log_line_count_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  log_lines_;

   public SystemServiceStatusMessage()
   {
      service_name_ = new java.lang.StringBuilder(255);
      status_ = new java.lang.StringBuilder(255);
      log_lines_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
   }

   public SystemServiceStatusMessage(SystemServiceStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemServiceStatusMessage other)
   {
      service_name_.setLength(0);
      service_name_.append(other.service_name_);

      status_.setLength(0);
      status_.append(other.status_);

      log_line_count_ = other.log_line_count_;

      log_lines_.set(other.log_lines_);
   }

   public void setServiceName(java.lang.String service_name)
   {
      service_name_.setLength(0);
      service_name_.append(service_name);
   }

   public java.lang.String getServiceNameAsString()
   {
      return getServiceName().toString();
   }
   public java.lang.StringBuilder getServiceName()
   {
      return service_name_;
   }

   public void setStatus(java.lang.String status)
   {
      status_.setLength(0);
      status_.append(status);
   }

   public java.lang.String getStatusAsString()
   {
      return getStatus().toString();
   }
   public java.lang.StringBuilder getStatus()
   {
      return status_;
   }

   public void setLogLineCount(int log_line_count)
   {
      log_line_count_ = log_line_count;
   }
   public int getLogLineCount()
   {
      return log_line_count_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getLogLines()
   {
      return log_lines_;
   }


   public static Supplier<SystemServiceStatusMessagePubSubType> getPubSubType()
   {
      return SystemServiceStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemServiceStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemServiceStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.service_name_, other.service_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.status_, other.status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.log_line_count_, other.log_line_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.log_lines_, other.log_lines_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemServiceStatusMessage)) return false;

      SystemServiceStatusMessage otherMyClass = (SystemServiceStatusMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.service_name_, otherMyClass.service_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.status_, otherMyClass.status_)) return false;

      if(this.log_line_count_ != otherMyClass.log_line_count_) return false;

      if (!this.log_lines_.equals(otherMyClass.log_lines_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemServiceStatusMessage {");
      builder.append("service_name=");
      builder.append(this.service_name_);      builder.append(", ");
      builder.append("status=");
      builder.append(this.status_);      builder.append(", ");
      builder.append("log_line_count=");
      builder.append(this.log_line_count_);      builder.append(", ");
      builder.append("log_lines=");
      builder.append(this.log_lines_);
      builder.append("}");
      return builder.toString();
   }
}
