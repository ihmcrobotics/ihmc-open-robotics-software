package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       * This message is used to allow the crutch to communicate a desired change in behavior.
       */
public class QuixCrutchMessage extends Packet<QuixCrutchMessage> implements Settable<QuixCrutchMessage>, EpsilonComparable<QuixCrutchMessage>
{
   public long sequence_id_;
   public boolean user_enable_;
   public boolean rewiggle_;
   public boolean start_behavior_;
   public controller_msgs.msg.dds.QuixMotionStateMessage requested_motion_state_;
   public boolean execute_behavior_;
   public boolean continuous_walking_;
   public controller_msgs.msg.dds.FlatStepTypeMessage flat_step_type_;
   public controller_msgs.msg.dds.QuixStairsStepTypeMessage stairs_step_type_;
   public controller_msgs.msg.dds.QuixSideStepDirectionMessage side_step_direction_;
   public controller_msgs.msg.dds.QuixSlopeStepTypeMessage slope_step_type_;
   public byte force_swing_side_ = (byte) 255;

   public QuixCrutchMessage()
   {
      requested_motion_state_ = new controller_msgs.msg.dds.QuixMotionStateMessage();
      flat_step_type_ = new controller_msgs.msg.dds.FlatStepTypeMessage();
      stairs_step_type_ = new controller_msgs.msg.dds.QuixStairsStepTypeMessage();
      side_step_direction_ = new controller_msgs.msg.dds.QuixSideStepDirectionMessage();
      slope_step_type_ = new controller_msgs.msg.dds.QuixSlopeStepTypeMessage();
   }

   public QuixCrutchMessage(QuixCrutchMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixCrutchMessage other)
   {
      sequence_id_ = other.sequence_id_;

      user_enable_ = other.user_enable_;

      rewiggle_ = other.rewiggle_;

      start_behavior_ = other.start_behavior_;

      controller_msgs.msg.dds.QuixMotionStateMessagePubSubType.staticCopy(other.requested_motion_state_, requested_motion_state_);
      execute_behavior_ = other.execute_behavior_;

      continuous_walking_ = other.continuous_walking_;

      controller_msgs.msg.dds.FlatStepTypeMessagePubSubType.staticCopy(other.flat_step_type_, flat_step_type_);
      controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType.staticCopy(other.stairs_step_type_, stairs_step_type_);
      controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType.staticCopy(other.side_step_direction_, side_step_direction_);
      controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType.staticCopy(other.slope_step_type_, slope_step_type_);
      force_swing_side_ = other.force_swing_side_;

   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setUserEnable(boolean user_enable)
   {
      user_enable_ = user_enable;
   }
   public boolean getUserEnable()
   {
      return user_enable_;
   }

   public void setRewiggle(boolean rewiggle)
   {
      rewiggle_ = rewiggle;
   }
   public boolean getRewiggle()
   {
      return rewiggle_;
   }

   public void setStartBehavior(boolean start_behavior)
   {
      start_behavior_ = start_behavior;
   }
   public boolean getStartBehavior()
   {
      return start_behavior_;
   }


   public controller_msgs.msg.dds.QuixMotionStateMessage getRequestedMotionState()
   {
      return requested_motion_state_;
   }

   public void setExecuteBehavior(boolean execute_behavior)
   {
      execute_behavior_ = execute_behavior;
   }
   public boolean getExecuteBehavior()
   {
      return execute_behavior_;
   }

   public void setContinuousWalking(boolean continuous_walking)
   {
      continuous_walking_ = continuous_walking;
   }
   public boolean getContinuousWalking()
   {
      return continuous_walking_;
   }


   public controller_msgs.msg.dds.FlatStepTypeMessage getFlatStepType()
   {
      return flat_step_type_;
   }


   public controller_msgs.msg.dds.QuixStairsStepTypeMessage getStairsStepType()
   {
      return stairs_step_type_;
   }


   public controller_msgs.msg.dds.QuixSideStepDirectionMessage getSideStepDirection()
   {
      return side_step_direction_;
   }


   public controller_msgs.msg.dds.QuixSlopeStepTypeMessage getSlopeStepType()
   {
      return slope_step_type_;
   }

   public void setForceSwingSide(byte force_swing_side)
   {
      force_swing_side_ = force_swing_side;
   }
   public byte getForceSwingSide()
   {
      return force_swing_side_;
   }


   public static Supplier<QuixCrutchMessagePubSubType> getPubSubType()
   {
      return QuixCrutchMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixCrutchMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixCrutchMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.user_enable_, other.user_enable_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.rewiggle_, other.rewiggle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.start_behavior_, other.start_behavior_, epsilon)) return false;

      if (!this.requested_motion_state_.epsilonEquals(other.requested_motion_state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_behavior_, other.execute_behavior_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.continuous_walking_, other.continuous_walking_, epsilon)) return false;

      if (!this.flat_step_type_.epsilonEquals(other.flat_step_type_, epsilon)) return false;
      if (!this.stairs_step_type_.epsilonEquals(other.stairs_step_type_, epsilon)) return false;
      if (!this.side_step_direction_.epsilonEquals(other.side_step_direction_, epsilon)) return false;
      if (!this.slope_step_type_.epsilonEquals(other.slope_step_type_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.force_swing_side_, other.force_swing_side_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixCrutchMessage)) return false;

      QuixCrutchMessage otherMyClass = (QuixCrutchMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.user_enable_ != otherMyClass.user_enable_) return false;

      if(this.rewiggle_ != otherMyClass.rewiggle_) return false;

      if(this.start_behavior_ != otherMyClass.start_behavior_) return false;

      if (!this.requested_motion_state_.equals(otherMyClass.requested_motion_state_)) return false;
      if(this.execute_behavior_ != otherMyClass.execute_behavior_) return false;

      if(this.continuous_walking_ != otherMyClass.continuous_walking_) return false;

      if (!this.flat_step_type_.equals(otherMyClass.flat_step_type_)) return false;
      if (!this.stairs_step_type_.equals(otherMyClass.stairs_step_type_)) return false;
      if (!this.side_step_direction_.equals(otherMyClass.side_step_direction_)) return false;
      if (!this.slope_step_type_.equals(otherMyClass.slope_step_type_)) return false;
      if(this.force_swing_side_ != otherMyClass.force_swing_side_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixCrutchMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("user_enable=");
      builder.append(this.user_enable_);      builder.append(", ");
      builder.append("rewiggle=");
      builder.append(this.rewiggle_);      builder.append(", ");
      builder.append("start_behavior=");
      builder.append(this.start_behavior_);      builder.append(", ");
      builder.append("requested_motion_state=");
      builder.append(this.requested_motion_state_);      builder.append(", ");
      builder.append("execute_behavior=");
      builder.append(this.execute_behavior_);      builder.append(", ");
      builder.append("continuous_walking=");
      builder.append(this.continuous_walking_);      builder.append(", ");
      builder.append("flat_step_type=");
      builder.append(this.flat_step_type_);      builder.append(", ");
      builder.append("stairs_step_type=");
      builder.append(this.stairs_step_type_);      builder.append(", ");
      builder.append("side_step_direction=");
      builder.append(this.side_step_direction_);      builder.append(", ");
      builder.append("slope_step_type=");
      builder.append(this.slope_step_type_);      builder.append(", ");
      builder.append("force_swing_side=");
      builder.append(this.force_swing_side_);
      builder.append("}");
      return builder.toString();
   }
}
