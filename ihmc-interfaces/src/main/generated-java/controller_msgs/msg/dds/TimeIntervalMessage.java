package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TimeIntervalMessage extends Packet<TimeIntervalMessage> implements Settable<TimeIntervalMessage>, EpsilonComparable<TimeIntervalMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public double start_time_;

   public double end_time_;

   public TimeIntervalMessage()
   {




   }

   public TimeIntervalMessage(TimeIntervalMessage other)
   {
      this();
      set(other);
   }

   public void set(TimeIntervalMessage other)
   {

      sequence_id_ = other.sequence_id_;


      start_time_ = other.start_time_;


      end_time_ = other.end_time_;

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


   public void setStartTime(double start_time)
   {
      start_time_ = start_time;
   }
   public double getStartTime()
   {
      return start_time_;
   }


   public void setEndTime(double end_time)
   {
      end_time_ = end_time;
   }
   public double getEndTime()
   {
      return end_time_;
   }


   public static Supplier<TimeIntervalMessagePubSubType> getPubSubType()
   {
      return TimeIntervalMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TimeIntervalMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TimeIntervalMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_time_, other.start_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_time_, other.end_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TimeIntervalMessage)) return false;

      TimeIntervalMessage otherMyClass = (TimeIntervalMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.start_time_ != otherMyClass.start_time_) return false;


      if(this.end_time_ != otherMyClass.end_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TimeIntervalMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("start_time=");
      builder.append(this.start_time_);      builder.append(", ");

      builder.append("end_time=");
      builder.append(this.end_time_);
      builder.append("}");
      return builder.toString();
   }
}
