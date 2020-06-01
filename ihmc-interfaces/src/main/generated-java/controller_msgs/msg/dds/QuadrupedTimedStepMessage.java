package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class QuadrupedTimedStepMessage extends Packet<QuadrupedTimedStepMessage> implements Settable<QuadrupedTimedStepMessage>, EpsilonComparable<QuadrupedTimedStepMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public controller_msgs.msg.dds.TimeIntervalMessage time_interval_;

   public controller_msgs.msg.dds.QuadrupedStepMessage quadruped_step_message_;

   public QuadrupedTimedStepMessage()
   {


      time_interval_ = new controller_msgs.msg.dds.TimeIntervalMessage();

      quadruped_step_message_ = new controller_msgs.msg.dds.QuadrupedStepMessage();

   }

   public QuadrupedTimedStepMessage(QuadrupedTimedStepMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTimedStepMessage other)
   {

      sequence_id_ = other.sequence_id_;


      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.staticCopy(other.time_interval_, time_interval_);

      controller_msgs.msg.dds.QuadrupedStepMessagePubSubType.staticCopy(other.quadruped_step_message_, quadruped_step_message_);
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



   public controller_msgs.msg.dds.TimeIntervalMessage getTimeInterval()
   {
      return time_interval_;
   }



   public controller_msgs.msg.dds.QuadrupedStepMessage getQuadrupedStepMessage()
   {
      return quadruped_step_message_;
   }


   public static Supplier<QuadrupedTimedStepMessagePubSubType> getPubSubType()
   {
      return QuadrupedTimedStepMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTimedStepMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTimedStepMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.time_interval_.epsilonEquals(other.time_interval_, epsilon)) return false;

      if (!this.quadruped_step_message_.epsilonEquals(other.quadruped_step_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTimedStepMessage)) return false;

      QuadrupedTimedStepMessage otherMyClass = (QuadrupedTimedStepMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.time_interval_.equals(otherMyClass.time_interval_)) return false;

      if (!this.quadruped_step_message_.equals(otherMyClass.quadruped_step_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTimedStepMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("time_interval=");
      builder.append(this.time_interval_);      builder.append(", ");

      builder.append("quadruped_step_message=");
      builder.append(this.quadruped_step_message_);
      builder.append("}");
      return builder.toString();
   }
}
