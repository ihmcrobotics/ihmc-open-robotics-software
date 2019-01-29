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
public class QuadrupedXGaitSettingsPacket extends Packet<QuadrupedXGaitSettingsPacket> implements Settable<QuadrupedXGaitSettingsPacket>, EpsilonComparable<QuadrupedXGaitSettingsPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double stance_length_ = -1.0;
   public double stance_width_ = -1.0;
   public double step_ground_clearance_ = -1.0;
   public double step_duration_ = -1.0;
   public double end_double_support_duration_ = -1.0;
   public double end_phase_shift_ = -1.0;

   public QuadrupedXGaitSettingsPacket()
   {
   }

   public QuadrupedXGaitSettingsPacket(QuadrupedXGaitSettingsPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedXGaitSettingsPacket other)
   {
      sequence_id_ = other.sequence_id_;

      stance_length_ = other.stance_length_;

      stance_width_ = other.stance_width_;

      step_ground_clearance_ = other.step_ground_clearance_;

      step_duration_ = other.step_duration_;

      end_double_support_duration_ = other.end_double_support_duration_;

      end_phase_shift_ = other.end_phase_shift_;

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

   public void setStanceLength(double stance_length)
   {
      stance_length_ = stance_length;
   }
   public double getStanceLength()
   {
      return stance_length_;
   }

   public void setStanceWidth(double stance_width)
   {
      stance_width_ = stance_width;
   }
   public double getStanceWidth()
   {
      return stance_width_;
   }

   public void setStepGroundClearance(double step_ground_clearance)
   {
      step_ground_clearance_ = step_ground_clearance;
   }
   public double getStepGroundClearance()
   {
      return step_ground_clearance_;
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

   public void setEndPhaseShift(double end_phase_shift)
   {
      end_phase_shift_ = end_phase_shift;
   }
   public double getEndPhaseShift()
   {
      return end_phase_shift_;
   }


   public static Supplier<QuadrupedXGaitSettingsPacketPubSubType> getPubSubType()
   {
      return QuadrupedXGaitSettingsPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedXGaitSettingsPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedXGaitSettingsPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_length_, other.stance_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_width_, other.stance_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_ground_clearance_, other.step_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_duration_, other.step_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_double_support_duration_, other.end_double_support_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_phase_shift_, other.end_phase_shift_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedXGaitSettingsPacket)) return false;

      QuadrupedXGaitSettingsPacket otherMyClass = (QuadrupedXGaitSettingsPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.stance_length_ != otherMyClass.stance_length_) return false;

      if(this.stance_width_ != otherMyClass.stance_width_) return false;

      if(this.step_ground_clearance_ != otherMyClass.step_ground_clearance_) return false;

      if(this.step_duration_ != otherMyClass.step_duration_) return false;

      if(this.end_double_support_duration_ != otherMyClass.end_double_support_duration_) return false;

      if(this.end_phase_shift_ != otherMyClass.end_phase_shift_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedXGaitSettingsPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("stance_length=");
      builder.append(this.stance_length_);      builder.append(", ");
      builder.append("stance_width=");
      builder.append(this.stance_width_);      builder.append(", ");
      builder.append("step_ground_clearance=");
      builder.append(this.step_ground_clearance_);      builder.append(", ");
      builder.append("step_duration=");
      builder.append(this.step_duration_);      builder.append(", ");
      builder.append("end_double_support_duration=");
      builder.append(this.end_double_support_duration_);      builder.append(", ");
      builder.append("end_phase_shift=");
      builder.append(this.end_phase_shift_);
      builder.append("}");
      return builder.toString();
   }
}
