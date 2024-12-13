package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is a request-reply ping used to estimate the clock offset
       * of another node to be used by data synchronization algorithms.
       */
public class DistributedClockMessage extends Packet<DistributedClockMessage> implements Settable<DistributedClockMessage>, EpsilonComparable<DistributedClockMessage>
{
   /**
            * The requester's unique ID
            */
   public java.lang.StringBuilder requester_id_;
   /**
            * The replier's unique ID
            */
   public java.lang.StringBuilder replier_id_;
   /**
            * Monotonically increasing request number
            */
   public long request_number_;
   /**
            * The time at which the request is sent
            */
   public ihmc_common_msgs.msg.dds.InstantMessage request_send_time_;
   /**
            * The time at which the reply is sent
            */
   public ihmc_common_msgs.msg.dds.InstantMessage reply_send_time_;

   public DistributedClockMessage()
   {
      requester_id_ = new java.lang.StringBuilder(255);
      replier_id_ = new java.lang.StringBuilder(255);
      request_send_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      reply_send_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
   }

   public DistributedClockMessage(DistributedClockMessage other)
   {
      this();
      set(other);
   }

   public void set(DistributedClockMessage other)
   {
      requester_id_.setLength(0);
      requester_id_.append(other.requester_id_);

      replier_id_.setLength(0);
      replier_id_.append(other.replier_id_);

      request_number_ = other.request_number_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.request_send_time_, request_send_time_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.reply_send_time_, reply_send_time_);
   }

   /**
            * The requester's unique ID
            */
   public void setRequesterId(java.lang.String requester_id)
   {
      requester_id_.setLength(0);
      requester_id_.append(requester_id);
   }

   /**
            * The requester's unique ID
            */
   public java.lang.String getRequesterIdAsString()
   {
      return getRequesterId().toString();
   }
   /**
            * The requester's unique ID
            */
   public java.lang.StringBuilder getRequesterId()
   {
      return requester_id_;
   }

   /**
            * The replier's unique ID
            */
   public void setReplierId(java.lang.String replier_id)
   {
      replier_id_.setLength(0);
      replier_id_.append(replier_id);
   }

   /**
            * The replier's unique ID
            */
   public java.lang.String getReplierIdAsString()
   {
      return getReplierId().toString();
   }
   /**
            * The replier's unique ID
            */
   public java.lang.StringBuilder getReplierId()
   {
      return replier_id_;
   }

   /**
            * Monotonically increasing request number
            */
   public void setRequestNumber(long request_number)
   {
      request_number_ = request_number;
   }
   /**
            * Monotonically increasing request number
            */
   public long getRequestNumber()
   {
      return request_number_;
   }


   /**
            * The time at which the request is sent
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getRequestSendTime()
   {
      return request_send_time_;
   }


   /**
            * The time at which the reply is sent
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getReplySendTime()
   {
      return reply_send_time_;
   }


   public static Supplier<DistributedClockMessagePubSubType> getPubSubType()
   {
      return DistributedClockMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DistributedClockMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DistributedClockMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.requester_id_, other.requester_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.replier_id_, other.replier_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.request_number_, other.request_number_, epsilon)) return false;

      if (!this.request_send_time_.epsilonEquals(other.request_send_time_, epsilon)) return false;
      if (!this.reply_send_time_.epsilonEquals(other.reply_send_time_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DistributedClockMessage)) return false;

      DistributedClockMessage otherMyClass = (DistributedClockMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.requester_id_, otherMyClass.requester_id_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.replier_id_, otherMyClass.replier_id_)) return false;

      if(this.request_number_ != otherMyClass.request_number_) return false;

      if (!this.request_send_time_.equals(otherMyClass.request_send_time_)) return false;
      if (!this.reply_send_time_.equals(otherMyClass.reply_send_time_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DistributedClockMessage {");
      builder.append("requester_id=");
      builder.append(this.requester_id_);      builder.append(", ");
      builder.append("replier_id=");
      builder.append(this.replier_id_);      builder.append(", ");
      builder.append("request_number=");
      builder.append(this.request_number_);      builder.append(", ");
      builder.append("request_send_time=");
      builder.append(this.request_send_time_);      builder.append(", ");
      builder.append("reply_send_time=");
      builder.append(this.reply_send_time_);
      builder.append("}");
      return builder.toString();
   }
}
