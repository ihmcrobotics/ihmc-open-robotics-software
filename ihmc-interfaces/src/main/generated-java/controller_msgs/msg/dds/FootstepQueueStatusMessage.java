package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message gives the status of the footstep queue in the controller in world coordinates.
       * See QueuedFootstepStatusMessage for more information about defining a footstep.
       */
public class FootstepQueueStatusMessage extends Packet<FootstepQueueStatusMessage> implements Settable<FootstepQueueStatusMessage>, EpsilonComparable<FootstepQueueStatusMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies whether the first step in the queue list is currently being taken in swing
            */
   public boolean is_first_step_in_swing_;
   /**
            * Specifies the time in the current support sequence of the current step. This could be in transfer, or in swing
            */
   public double time_in_support_sequence_;
   /**
            * Defines the list of footsteps contained in the queue.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueuedFootstepStatusMessage>  queued_footstep_list_;

   public FootstepQueueStatusMessage()
   {
      queued_footstep_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueuedFootstepStatusMessage> (50, new controller_msgs.msg.dds.QueuedFootstepStatusMessagePubSubType());

   }

   public FootstepQueueStatusMessage(FootstepQueueStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepQueueStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      is_first_step_in_swing_ = other.is_first_step_in_swing_;

      time_in_support_sequence_ = other.time_in_support_sequence_;

      queued_footstep_list_.set(other.queued_footstep_list_);
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

   /**
            * Specifies whether the first step in the queue list is currently being taken in swing
            */
   public void setIsFirstStepInSwing(boolean is_first_step_in_swing)
   {
      is_first_step_in_swing_ = is_first_step_in_swing;
   }
   /**
            * Specifies whether the first step in the queue list is currently being taken in swing
            */
   public boolean getIsFirstStepInSwing()
   {
      return is_first_step_in_swing_;
   }

   /**
            * Specifies the time in the current support sequence of the current step. This could be in transfer, or in swing
            */
   public void setTimeInSupportSequence(double time_in_support_sequence)
   {
      time_in_support_sequence_ = time_in_support_sequence;
   }
   /**
            * Specifies the time in the current support sequence of the current step. This could be in transfer, or in swing
            */
   public double getTimeInSupportSequence()
   {
      return time_in_support_sequence_;
   }


   /**
            * Defines the list of footsteps contained in the queue.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueuedFootstepStatusMessage>  getQueuedFootstepList()
   {
      return queued_footstep_list_;
   }


   public static Supplier<FootstepQueueStatusMessagePubSubType> getPubSubType()
   {
      return FootstepQueueStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepQueueStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepQueueStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_first_step_in_swing_, other.is_first_step_in_swing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_in_support_sequence_, other.time_in_support_sequence_, epsilon)) return false;

      if (this.queued_footstep_list_.size() != other.queued_footstep_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.queued_footstep_list_.size(); i++)
         {  if (!this.queued_footstep_list_.get(i).epsilonEquals(other.queued_footstep_list_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepQueueStatusMessage)) return false;

      FootstepQueueStatusMessage otherMyClass = (FootstepQueueStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.is_first_step_in_swing_ != otherMyClass.is_first_step_in_swing_) return false;

      if(this.time_in_support_sequence_ != otherMyClass.time_in_support_sequence_) return false;

      if (!this.queued_footstep_list_.equals(otherMyClass.queued_footstep_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepQueueStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("is_first_step_in_swing=");
      builder.append(this.is_first_step_in_swing_);      builder.append(", ");
      builder.append("time_in_support_sequence=");
      builder.append(this.time_in_support_sequence_);      builder.append(", ");
      builder.append("queued_footstep_list=");
      builder.append(this.queued_footstep_list_);
      builder.append("}");
      return builder.toString();
   }
}
