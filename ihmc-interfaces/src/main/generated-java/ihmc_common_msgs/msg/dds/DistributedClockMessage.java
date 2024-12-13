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
            * The guid of the publisher that's expected to send back a reply
            */
   public ihmc_common_msgs.msg.dds.GuidMessage request_target_;
   /**
            * The guid of the publisher associated with the requester
            */
   public ihmc_common_msgs.msg.dds.GuidMessage reply_target_;
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
      request_target_ = new ihmc_common_msgs.msg.dds.GuidMessage();
      reply_target_ = new ihmc_common_msgs.msg.dds.GuidMessage();
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
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.staticCopy(other.request_target_, request_target_);
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.staticCopy(other.reply_target_, reply_target_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.request_send_time_, request_send_time_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.reply_send_time_, reply_send_time_);
   }


   /**
            * The guid of the publisher that's expected to send back a reply
            */
   public ihmc_common_msgs.msg.dds.GuidMessage getRequestTarget()
   {
      return request_target_;
   }


   /**
            * The guid of the publisher associated with the requester
            */
   public ihmc_common_msgs.msg.dds.GuidMessage getReplyTarget()
   {
      return reply_target_;
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

      if (!this.request_target_.epsilonEquals(other.request_target_, epsilon)) return false;
      if (!this.reply_target_.epsilonEquals(other.reply_target_, epsilon)) return false;
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

      if (!this.request_target_.equals(otherMyClass.request_target_)) return false;
      if (!this.reply_target_.equals(otherMyClass.reply_target_)) return false;
      if (!this.request_send_time_.equals(otherMyClass.request_send_time_)) return false;
      if (!this.reply_send_time_.equals(otherMyClass.reply_send_time_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DistributedClockMessage {");
      builder.append("request_target=");
      builder.append(this.request_target_);      builder.append(", ");
      builder.append("reply_target=");
      builder.append(this.reply_target_);      builder.append(", ");
      builder.append("request_send_time=");
      builder.append(this.request_send_time_);      builder.append(", ");
      builder.append("reply_send_time=");
      builder.append(this.reply_send_time_);
      builder.append("}");
      return builder.toString();
   }
}
