package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemServiceLogRefreshMessage extends Packet<SystemServiceLogRefreshMessage> implements Settable<SystemServiceLogRefreshMessage>, EpsilonComparable<SystemServiceLogRefreshMessage>
{
   /**
            * The name of the systemd service
            */
   public java.lang.StringBuilder service_name_;

   public SystemServiceLogRefreshMessage()
   {
      service_name_ = new java.lang.StringBuilder(255);
   }

   public SystemServiceLogRefreshMessage(SystemServiceLogRefreshMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemServiceLogRefreshMessage other)
   {
      service_name_.setLength(0);
      service_name_.append(other.service_name_);
   }

   /**
            * The name of the systemd service
            */
   public void setServiceName(java.lang.String service_name)
   {
      service_name_.setLength(0);
      service_name_.append(service_name);
   }

   /**
            * The name of the systemd service
            */
   public java.lang.String getServiceNameAsString()
   {
      return getServiceName().toString();
   }
   /**
            * The name of the systemd service
            */
   public java.lang.StringBuilder getServiceName()
   {
      return service_name_;
   }


   public static Supplier<SystemServiceLogRefreshMessagePubSubType> getPubSubType()
   {
      return SystemServiceLogRefreshMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemServiceLogRefreshMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemServiceLogRefreshMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.service_name_, other.service_name_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemServiceLogRefreshMessage)) return false;

      SystemServiceLogRefreshMessage otherMyClass = (SystemServiceLogRefreshMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.service_name_, otherMyClass.service_name_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemServiceLogRefreshMessage {");
      builder.append("service_name=");
      builder.append(this.service_name_);
      builder.append("}");
      return builder.toString();
   }
}
