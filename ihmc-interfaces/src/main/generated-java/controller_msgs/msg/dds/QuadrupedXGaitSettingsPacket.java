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
   public static final byte SLOW = (byte) 0;
   public static final byte MEDIUM = (byte) 1;
   public static final byte FAST = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double end_phase_shift_ = -1.0;
   public double stance_length_ = -1.0;
   public double stance_width_ = -1.0;
   public double step_ground_clearance_ = -1.0;
   public double max_horizontal_speed_fraction_ = -1.0;
   public double max_yaw_speed_fraction_ = -1.0;
   public byte quadruped_speed_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket pace_slow_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket pace_medium_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket pace_fast_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket amble_slow_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket amble_medium_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket amble_fast_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket trot_slow_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket trot_medium_settings_packet_;
   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket trot_fast_settings_packet_;

   public QuadrupedXGaitSettingsPacket()
   {
      pace_slow_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      pace_medium_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      pace_fast_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      amble_slow_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      amble_medium_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      amble_fast_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      trot_slow_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      trot_medium_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
      trot_fast_settings_packet_ = new controller_msgs.msg.dds.QuadrupedGaitTimingsPacket();
   }

   public QuadrupedXGaitSettingsPacket(QuadrupedXGaitSettingsPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedXGaitSettingsPacket other)
   {
      sequence_id_ = other.sequence_id_;

      end_phase_shift_ = other.end_phase_shift_;

      stance_length_ = other.stance_length_;

      stance_width_ = other.stance_width_;

      step_ground_clearance_ = other.step_ground_clearance_;

      max_horizontal_speed_fraction_ = other.max_horizontal_speed_fraction_;

      max_yaw_speed_fraction_ = other.max_yaw_speed_fraction_;

      quadruped_speed_ = other.quadruped_speed_;

      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.pace_slow_settings_packet_, pace_slow_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.pace_medium_settings_packet_, pace_medium_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.pace_fast_settings_packet_, pace_fast_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.amble_slow_settings_packet_, amble_slow_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.amble_medium_settings_packet_, amble_medium_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.amble_fast_settings_packet_, amble_fast_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.trot_slow_settings_packet_, trot_slow_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.trot_medium_settings_packet_, trot_medium_settings_packet_);
      controller_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.staticCopy(other.trot_fast_settings_packet_, trot_fast_settings_packet_);
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

   public void setEndPhaseShift(double end_phase_shift)
   {
      end_phase_shift_ = end_phase_shift;
   }
   public double getEndPhaseShift()
   {
      return end_phase_shift_;
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

   public void setMaxHorizontalSpeedFraction(double max_horizontal_speed_fraction)
   {
      max_horizontal_speed_fraction_ = max_horizontal_speed_fraction;
   }
   public double getMaxHorizontalSpeedFraction()
   {
      return max_horizontal_speed_fraction_;
   }

   public void setMaxYawSpeedFraction(double max_yaw_speed_fraction)
   {
      max_yaw_speed_fraction_ = max_yaw_speed_fraction;
   }
   public double getMaxYawSpeedFraction()
   {
      return max_yaw_speed_fraction_;
   }

   public void setQuadrupedSpeed(byte quadruped_speed)
   {
      quadruped_speed_ = quadruped_speed;
   }
   public byte getQuadrupedSpeed()
   {
      return quadruped_speed_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getPaceSlowSettingsPacket()
   {
      return pace_slow_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getPaceMediumSettingsPacket()
   {
      return pace_medium_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getPaceFastSettingsPacket()
   {
      return pace_fast_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getAmbleSlowSettingsPacket()
   {
      return amble_slow_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getAmbleMediumSettingsPacket()
   {
      return amble_medium_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getAmbleFastSettingsPacket()
   {
      return amble_fast_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getTrotSlowSettingsPacket()
   {
      return trot_slow_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getTrotMediumSettingsPacket()
   {
      return trot_medium_settings_packet_;
   }


   public controller_msgs.msg.dds.QuadrupedGaitTimingsPacket getTrotFastSettingsPacket()
   {
      return trot_fast_settings_packet_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_phase_shift_, other.end_phase_shift_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_length_, other.stance_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_width_, other.stance_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_ground_clearance_, other.step_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_horizontal_speed_fraction_, other.max_horizontal_speed_fraction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_yaw_speed_fraction_, other.max_yaw_speed_fraction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quadruped_speed_, other.quadruped_speed_, epsilon)) return false;

      if (!this.pace_slow_settings_packet_.epsilonEquals(other.pace_slow_settings_packet_, epsilon)) return false;
      if (!this.pace_medium_settings_packet_.epsilonEquals(other.pace_medium_settings_packet_, epsilon)) return false;
      if (!this.pace_fast_settings_packet_.epsilonEquals(other.pace_fast_settings_packet_, epsilon)) return false;
      if (!this.amble_slow_settings_packet_.epsilonEquals(other.amble_slow_settings_packet_, epsilon)) return false;
      if (!this.amble_medium_settings_packet_.epsilonEquals(other.amble_medium_settings_packet_, epsilon)) return false;
      if (!this.amble_fast_settings_packet_.epsilonEquals(other.amble_fast_settings_packet_, epsilon)) return false;
      if (!this.trot_slow_settings_packet_.epsilonEquals(other.trot_slow_settings_packet_, epsilon)) return false;
      if (!this.trot_medium_settings_packet_.epsilonEquals(other.trot_medium_settings_packet_, epsilon)) return false;
      if (!this.trot_fast_settings_packet_.epsilonEquals(other.trot_fast_settings_packet_, epsilon)) return false;

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

      if(this.end_phase_shift_ != otherMyClass.end_phase_shift_) return false;

      if(this.stance_length_ != otherMyClass.stance_length_) return false;

      if(this.stance_width_ != otherMyClass.stance_width_) return false;

      if(this.step_ground_clearance_ != otherMyClass.step_ground_clearance_) return false;

      if(this.max_horizontal_speed_fraction_ != otherMyClass.max_horizontal_speed_fraction_) return false;

      if(this.max_yaw_speed_fraction_ != otherMyClass.max_yaw_speed_fraction_) return false;

      if(this.quadruped_speed_ != otherMyClass.quadruped_speed_) return false;

      if (!this.pace_slow_settings_packet_.equals(otherMyClass.pace_slow_settings_packet_)) return false;
      if (!this.pace_medium_settings_packet_.equals(otherMyClass.pace_medium_settings_packet_)) return false;
      if (!this.pace_fast_settings_packet_.equals(otherMyClass.pace_fast_settings_packet_)) return false;
      if (!this.amble_slow_settings_packet_.equals(otherMyClass.amble_slow_settings_packet_)) return false;
      if (!this.amble_medium_settings_packet_.equals(otherMyClass.amble_medium_settings_packet_)) return false;
      if (!this.amble_fast_settings_packet_.equals(otherMyClass.amble_fast_settings_packet_)) return false;
      if (!this.trot_slow_settings_packet_.equals(otherMyClass.trot_slow_settings_packet_)) return false;
      if (!this.trot_medium_settings_packet_.equals(otherMyClass.trot_medium_settings_packet_)) return false;
      if (!this.trot_fast_settings_packet_.equals(otherMyClass.trot_fast_settings_packet_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedXGaitSettingsPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("end_phase_shift=");
      builder.append(this.end_phase_shift_);      builder.append(", ");
      builder.append("stance_length=");
      builder.append(this.stance_length_);      builder.append(", ");
      builder.append("stance_width=");
      builder.append(this.stance_width_);      builder.append(", ");
      builder.append("step_ground_clearance=");
      builder.append(this.step_ground_clearance_);      builder.append(", ");
      builder.append("max_horizontal_speed_fraction=");
      builder.append(this.max_horizontal_speed_fraction_);      builder.append(", ");
      builder.append("max_yaw_speed_fraction=");
      builder.append(this.max_yaw_speed_fraction_);      builder.append(", ");
      builder.append("quadruped_speed=");
      builder.append(this.quadruped_speed_);      builder.append(", ");
      builder.append("pace_slow_settings_packet=");
      builder.append(this.pace_slow_settings_packet_);      builder.append(", ");
      builder.append("pace_medium_settings_packet=");
      builder.append(this.pace_medium_settings_packet_);      builder.append(", ");
      builder.append("pace_fast_settings_packet=");
      builder.append(this.pace_fast_settings_packet_);      builder.append(", ");
      builder.append("amble_slow_settings_packet=");
      builder.append(this.amble_slow_settings_packet_);      builder.append(", ");
      builder.append("amble_medium_settings_packet=");
      builder.append(this.amble_medium_settings_packet_);      builder.append(", ");
      builder.append("amble_fast_settings_packet=");
      builder.append(this.amble_fast_settings_packet_);      builder.append(", ");
      builder.append("trot_slow_settings_packet=");
      builder.append(this.trot_slow_settings_packet_);      builder.append(", ");
      builder.append("trot_medium_settings_packet=");
      builder.append(this.trot_medium_settings_packet_);      builder.append(", ");
      builder.append("trot_fast_settings_packet=");
      builder.append(this.trot_fast_settings_packet_);
      builder.append("}");
      return builder.toString();
   }
}
