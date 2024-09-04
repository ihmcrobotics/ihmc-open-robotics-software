package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SRTStreamRequest extends Packet<SRTStreamRequest> implements Settable<SRTStreamRequest>, EpsilonComparable<SRTStreamRequest>
{
   /**
            * Name of the topic being requested
            */
   public java.lang.StringBuilder topic_name_;
   /**
            * Address to send the stream to
            */
   public java.lang.StringBuilder receiver_address_;
   /**
            * Port to send the stream to
            */
   public int receiver_port_;

   public SRTStreamRequest()
   {
      topic_name_ = new java.lang.StringBuilder(255);
      receiver_address_ = new java.lang.StringBuilder(255);
   }

   public SRTStreamRequest(SRTStreamRequest other)
   {
      this();
      set(other);
   }

   public void set(SRTStreamRequest other)
   {
      topic_name_.setLength(0);
      topic_name_.append(other.topic_name_);

      receiver_address_.setLength(0);
      receiver_address_.append(other.receiver_address_);

      receiver_port_ = other.receiver_port_;

   }

   /**
            * Name of the topic being requested
            */
   public void setTopicName(java.lang.String topic_name)
   {
      topic_name_.setLength(0);
      topic_name_.append(topic_name);
   }

   /**
            * Name of the topic being requested
            */
   public java.lang.String getTopicNameAsString()
   {
      return getTopicName().toString();
   }
   /**
            * Name of the topic being requested
            */
   public java.lang.StringBuilder getTopicName()
   {
      return topic_name_;
   }

   /**
            * Address to send the stream to
            */
   public void setReceiverAddress(java.lang.String receiver_address)
   {
      receiver_address_.setLength(0);
      receiver_address_.append(receiver_address);
   }

   /**
            * Address to send the stream to
            */
   public java.lang.String getReceiverAddressAsString()
   {
      return getReceiverAddress().toString();
   }
   /**
            * Address to send the stream to
            */
   public java.lang.StringBuilder getReceiverAddress()
   {
      return receiver_address_;
   }

   /**
            * Port to send the stream to
            */
   public void setReceiverPort(int receiver_port)
   {
      receiver_port_ = receiver_port;
   }
   /**
            * Port to send the stream to
            */
   public int getReceiverPort()
   {
      return receiver_port_;
   }


   public static Supplier<SRTStreamRequestPubSubType> getPubSubType()
   {
      return SRTStreamRequestPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SRTStreamRequestPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SRTStreamRequest other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.topic_name_, other.topic_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.receiver_address_, other.receiver_address_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.receiver_port_, other.receiver_port_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SRTStreamRequest)) return false;

      SRTStreamRequest otherMyClass = (SRTStreamRequest) other;

      if (!us.ihmc.idl.IDLTools.equals(this.topic_name_, otherMyClass.topic_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.receiver_address_, otherMyClass.receiver_address_)) return false;

      if(this.receiver_port_ != otherMyClass.receiver_port_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SRTStreamRequest {");
      builder.append("topic_name=");
      builder.append(this.topic_name_);      builder.append(", ");
      builder.append("receiver_address=");
      builder.append(this.receiver_address_);      builder.append(", ");
      builder.append("receiver_port=");
      builder.append(this.receiver_port_);
      builder.append("}");
      return builder.toString();
   }
}
