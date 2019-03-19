package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message sends the x gait settings used for determining gait.
       */
public class QuadrupedGaitTimingsPacket extends Packet<QuadrupedGaitTimingsPacket> implements Settable<QuadrupedGaitTimingsPacket>, EpsilonComparable<QuadrupedGaitTimingsPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double step_duration_ = -1.0;
   public double end_double_support_duration_ = -1.0;
   public double max_speed_ = -1.0;

   public QuadrupedGaitTimingsPacket()
   {
   }

   public QuadrupedGaitTimingsPacket(QuadrupedGaitTimingsPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedGaitTimingsPacket other)
   {
      sequence_id_ = other.sequence_id_;

      step_duration_ = other.step_duration_;

      end_double_support_duration_ = other.end_double_support_duration_;

      max_speed_ = other.max_speed_;

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

   public void setStepDuration(double step_duration)
   {
      step_duration_ = step_duration;
   }
   public double getStepDuration()
   {
      return step_duration_;
   }

   public void setEndDoubleSupportDuration(double end_double_support_duration)
   {
      end_double_support_duration_ = end_double_support_duration;
   }
   public double getEndDoubleSupportDuration()
   {
      return end_double_support_duration_;
   }

   public void setMaxSpeed(double max_speed)
   {
      max_speed_ = max_speed;
   }
   public double getMaxSpeed()
   {
      return max_speed_;
   }


   public static Supplier<QuadrupedGaitTimingsPacketPubSubType> getPubSubType()
   {
      return QuadrupedGaitTimingsPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedGaitTimingsPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedGaitTimingsPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_duration_, other.step_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_double_support_duration_, other.end_double_support_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_speed_, other.max_speed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedGaitTimingsPacket)) return false;

      QuadrupedGaitTimingsPacket otherMyClass = (QuadrupedGaitTimingsPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.step_duration_ != otherMyClass.step_duration_) return false;

      if(this.end_double_support_duration_ != otherMyClass.end_double_support_duration_) return false;

      if(this.max_speed_ != otherMyClass.max_speed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedGaitTimingsPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("step_duration=");
      builder.append(this.step_duration_);      builder.append(", ");
      builder.append("end_double_support_duration=");
      builder.append(this.end_double_support_duration_);      builder.append(", ");
      builder.append("max_speed=");
      builder.append(this.max_speed_);
      builder.append("}");
      return builder.toString();
   }
}
