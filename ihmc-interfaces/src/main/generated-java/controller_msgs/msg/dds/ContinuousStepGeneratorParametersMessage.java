package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for the ContinuousStepGenerator to configure the module's parameters.
       * This module is used for controlling the walking gait using a joystick controller.
       */
public class ContinuousStepGeneratorParametersMessage extends Packet<ContinuousStepGeneratorParametersMessage> implements Settable<ContinuousStepGeneratorParametersMessage>, EpsilonComparable<ContinuousStepGeneratorParametersMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public int number_of_footsteps_to_plan_ = -1;
   public int number_of_fixed_footsteps_ = -1;
   public double swing_height_ = -1.0;
   public double swing_duration_ = -1.0;
   public double transfer_duration_ = -1.0;
   public double max_step_length_ = -1.0;
   public double default_step_width_ = -1.0;
   public double min_step_width_ = -1.0;
   public double max_step_width_ = -1.0;
   public double turn_max_angle_inward_ = -1.0;
   public double turn_max_angle_outward_ = -1.0;

   public ContinuousStepGeneratorParametersMessage()
   {
   }

   public ContinuousStepGeneratorParametersMessage(ContinuousStepGeneratorParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousStepGeneratorParametersMessage other)
   {
      sequence_id_ = other.sequence_id_;

      number_of_footsteps_to_plan_ = other.number_of_footsteps_to_plan_;

      number_of_fixed_footsteps_ = other.number_of_fixed_footsteps_;

      swing_height_ = other.swing_height_;

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      max_step_length_ = other.max_step_length_;

      default_step_width_ = other.default_step_width_;

      min_step_width_ = other.min_step_width_;

      max_step_width_ = other.max_step_width_;

      turn_max_angle_inward_ = other.turn_max_angle_inward_;

      turn_max_angle_outward_ = other.turn_max_angle_outward_;

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

   public void setNumberOfFootstepsToPlan(int number_of_footsteps_to_plan)
   {
      number_of_footsteps_to_plan_ = number_of_footsteps_to_plan;
   }
   public int getNumberOfFootstepsToPlan()
   {
      return number_of_footsteps_to_plan_;
   }

   public void setNumberOfFixedFootsteps(int number_of_fixed_footsteps)
   {
      number_of_fixed_footsteps_ = number_of_fixed_footsteps;
   }
   public int getNumberOfFixedFootsteps()
   {
      return number_of_fixed_footsteps_;
   }

   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }
   public double getSwingHeight()
   {
      return swing_height_;
   }

   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   public void setMaxStepLength(double max_step_length)
   {
      max_step_length_ = max_step_length;
   }
   public double getMaxStepLength()
   {
      return max_step_length_;
   }

   public void setDefaultStepWidth(double default_step_width)
   {
      default_step_width_ = default_step_width;
   }
   public double getDefaultStepWidth()
   {
      return default_step_width_;
   }

   public void setMinStepWidth(double min_step_width)
   {
      min_step_width_ = min_step_width;
   }
   public double getMinStepWidth()
   {
      return min_step_width_;
   }

   public void setMaxStepWidth(double max_step_width)
   {
      max_step_width_ = max_step_width;
   }
   public double getMaxStepWidth()
   {
      return max_step_width_;
   }

   public void setTurnMaxAngleInward(double turn_max_angle_inward)
   {
      turn_max_angle_inward_ = turn_max_angle_inward;
   }
   public double getTurnMaxAngleInward()
   {
      return turn_max_angle_inward_;
   }

   public void setTurnMaxAngleOutward(double turn_max_angle_outward)
   {
      turn_max_angle_outward_ = turn_max_angle_outward;
   }
   public double getTurnMaxAngleOutward()
   {
      return turn_max_angle_outward_;
   }


   public static Supplier<ContinuousStepGeneratorParametersMessagePubSubType> getPubSubType()
   {
      return ContinuousStepGeneratorParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousStepGeneratorParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousStepGeneratorParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_footsteps_to_plan_, other.number_of_footsteps_to_plan_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_fixed_footsteps_, other.number_of_fixed_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_step_length_, other.max_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_step_width_, other.default_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_step_width_, other.min_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_step_width_, other.max_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turn_max_angle_inward_, other.turn_max_angle_inward_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turn_max_angle_outward_, other.turn_max_angle_outward_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousStepGeneratorParametersMessage)) return false;

      ContinuousStepGeneratorParametersMessage otherMyClass = (ContinuousStepGeneratorParametersMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.number_of_footsteps_to_plan_ != otherMyClass.number_of_footsteps_to_plan_) return false;

      if(this.number_of_fixed_footsteps_ != otherMyClass.number_of_fixed_footsteps_) return false;

      if(this.swing_height_ != otherMyClass.swing_height_) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if(this.max_step_length_ != otherMyClass.max_step_length_) return false;

      if(this.default_step_width_ != otherMyClass.default_step_width_) return false;

      if(this.min_step_width_ != otherMyClass.min_step_width_) return false;

      if(this.max_step_width_ != otherMyClass.max_step_width_) return false;

      if(this.turn_max_angle_inward_ != otherMyClass.turn_max_angle_inward_) return false;

      if(this.turn_max_angle_outward_ != otherMyClass.turn_max_angle_outward_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousStepGeneratorParametersMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("number_of_footsteps_to_plan=");
      builder.append(this.number_of_footsteps_to_plan_);      builder.append(", ");
      builder.append("number_of_fixed_footsteps=");
      builder.append(this.number_of_fixed_footsteps_);      builder.append(", ");
      builder.append("swing_height=");
      builder.append(this.swing_height_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("max_step_length=");
      builder.append(this.max_step_length_);      builder.append(", ");
      builder.append("default_step_width=");
      builder.append(this.default_step_width_);      builder.append(", ");
      builder.append("min_step_width=");
      builder.append(this.min_step_width_);      builder.append(", ");
      builder.append("max_step_width=");
      builder.append(this.max_step_width_);      builder.append(", ");
      builder.append("turn_max_angle_inward=");
      builder.append(this.turn_max_angle_inward_);      builder.append(", ");
      builder.append("turn_max_angle_outward=");
      builder.append(this.turn_max_angle_outward_);
      builder.append("}");
      return builder.toString();
   }
}
