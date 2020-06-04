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

   public QuixCrutchMessage()
   {





      requested_motion_state_ = new controller_msgs.msg.dds.QuixMotionStateMessage();


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
      builder.append(this.execute_behavior_);
      builder.append("}");
      return builder.toString();
   }
}
