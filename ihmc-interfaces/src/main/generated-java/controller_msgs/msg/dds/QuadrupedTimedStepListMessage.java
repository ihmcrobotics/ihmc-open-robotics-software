package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class QuadrupedTimedStepListMessage extends Packet<QuadrupedTimedStepListMessage> implements Settable<QuadrupedTimedStepListMessage>, EpsilonComparable<QuadrupedTimedStepListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Defines the list of steps to perform
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QuadrupedTimedStepMessage>  quadruped_step_list_;
   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public boolean is_expressed_in_absolute_time_ = true;
   /**
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;
   /**
            * Defines if the step list is adjustable
            */
   public boolean are_steps_adjustable_ = true;

   public QuadrupedTimedStepListMessage()
   {
      quadruped_step_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QuadrupedTimedStepMessage> (100, new controller_msgs.msg.dds.QuadrupedTimedStepMessagePubSubType());
      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();

   }

   public QuadrupedTimedStepListMessage(QuadrupedTimedStepListMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTimedStepListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      quadruped_step_list_.set(other.quadruped_step_list_);
      is_expressed_in_absolute_time_ = other.is_expressed_in_absolute_time_;

      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
      are_steps_adjustable_ = other.are_steps_adjustable_;

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
            * Defines the list of steps to perform
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QuadrupedTimedStepMessage>  getQuadrupedStepList()
   {
      return quadruped_step_list_;
   }

   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public void setIsExpressedInAbsoluteTime(boolean is_expressed_in_absolute_time)
   {
      is_expressed_in_absolute_time_ = is_expressed_in_absolute_time;
   }
   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public boolean getIsExpressedInAbsoluteTime()
   {
      return is_expressed_in_absolute_time_;
   }


   /**
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }

   /**
            * Defines if the step list is adjustable
            */
   public void setAreStepsAdjustable(boolean are_steps_adjustable)
   {
      are_steps_adjustable_ = are_steps_adjustable;
   }
   /**
            * Defines if the step list is adjustable
            */
   public boolean getAreStepsAdjustable()
   {
      return are_steps_adjustable_;
   }


   public static Supplier<QuadrupedTimedStepListMessagePubSubType> getPubSubType()
   {
      return QuadrupedTimedStepListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTimedStepListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTimedStepListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.quadruped_step_list_.size() != other.quadruped_step_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.quadruped_step_list_.size(); i++)
         {  if (!this.quadruped_step_list_.get(i).epsilonEquals(other.quadruped_step_list_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_expressed_in_absolute_time_, other.is_expressed_in_absolute_time_, epsilon)) return false;

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_steps_adjustable_, other.are_steps_adjustable_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTimedStepListMessage)) return false;

      QuadrupedTimedStepListMessage otherMyClass = (QuadrupedTimedStepListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.quadruped_step_list_.equals(otherMyClass.quadruped_step_list_)) return false;
      if(this.is_expressed_in_absolute_time_ != otherMyClass.is_expressed_in_absolute_time_) return false;

      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;
      if(this.are_steps_adjustable_ != otherMyClass.are_steps_adjustable_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTimedStepListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("quadruped_step_list=");
      builder.append(this.quadruped_step_list_);      builder.append(", ");
      builder.append("is_expressed_in_absolute_time=");
      builder.append(this.is_expressed_in_absolute_time_);      builder.append(", ");
      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);      builder.append(", ");
      builder.append("are_steps_adjustable=");
      builder.append(this.are_steps_adjustable_);
      builder.append("}");
      return builder.toString();
   }
}
