package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to execute a list of footsteps.
       * See FootstepDataMessage for more information about defining a footstep.
       */
public class FootstepDataListMessage extends Packet<FootstepDataListMessage> implements Settable<FootstepDataListMessage>, EpsilonComparable<FootstepDataListMessage>
{
   public static final byte EXECUTION_TIMING_CONTROL_DURATIONS = (byte) 0;
   public static final byte EXECUTION_TIMING_CONTROL_ABSOLUTE_TIMINGS = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  footstep_data_list_;
   /**
            * When CONTROL_DURATIONS is chosen:
            * The controller will try to achieve the swing_duration and the transfer_duration specified in the message.
            * If a footstep touches down early, the next step will not be affected by this and the whole trajectory might finish earlier than expected.
            * When CONTROL_ABSOLUTE_TIMINGS is chosen:
            * The controller will compute the expected times for swing start and touchdown and attempt to start a footstep at that time.
            * If a footstep touches down early, the following transfer will be extended to make up for this
            * time difference and the footstep plan will finish at the expected time.
            */
   public byte execution_timing_;
   /**
            * The swing_duration is the time a foot is not in ground contact during a step.
            * Each step in a list of footsteps might have a different swing duration.
            * The value specified here is a default value, used if a footstep in this list was created without a swing_duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public double default_swing_duration_ = -1.0;
   /**
            * The transfer_duration is the time spent with the feet in ground contact before a step.
            * Each step in a list of footsteps might have a different transfer duration.
            * The value specified here is a default value, used if a footstep in this list was created without a transfer-duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public double default_transfer_duration_ = -1.0;
   /**
            * Specifies the time used to return to a stable standing stance after the execution of the
            * footstep list is finished. If the value is negative the default_transfer_duration will be used,
            * which in turn if not provided indicate the controller to use its own internal default value.
            */
   public double final_transfer_duration_ = -1.0;
   /**
            * If false the controller adjust each footstep height to be at the support sole height.
            */
   public boolean trust_height_of_footsteps_ = true;
   /**
            * Contains information on whether the robot can automatically adjust its footsteps to retain balance.
            */
   public boolean are_footsteps_adjustable_;
   /**
            * If true the controller will adjust upcoming footsteps with the location error of previous steps.
            */
   public boolean offset_footsteps_with_execution_error_;
   /**
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;

   public FootstepDataListMessage()
   {
      footstep_data_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> (1, controller_msgs.msg.dds.FootstepDataMessage.class, new controller_msgs.msg.dds.FootstepDataMessagePubSubType());
      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();

   }

   public FootstepDataListMessage(FootstepDataListMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepDataListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      footstep_data_list_.set(other.footstep_data_list_);
      execution_timing_ = other.execution_timing_;

      default_swing_duration_ = other.default_swing_duration_;

      default_transfer_duration_ = other.default_transfer_duration_;

      final_transfer_duration_ = other.final_transfer_duration_;

      trust_height_of_footsteps_ = other.trust_height_of_footsteps_;

      are_footsteps_adjustable_ = other.are_footsteps_adjustable_;

      offset_footsteps_with_execution_error_ = other.offset_footsteps_with_execution_error_;

      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
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
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  getFootstepDataList()
   {
      return footstep_data_list_;
   }

   /**
            * When CONTROL_DURATIONS is chosen:
            * The controller will try to achieve the swing_duration and the transfer_duration specified in the message.
            * If a footstep touches down early, the next step will not be affected by this and the whole trajectory might finish earlier than expected.
            * When CONTROL_ABSOLUTE_TIMINGS is chosen:
            * The controller will compute the expected times for swing start and touchdown and attempt to start a footstep at that time.
            * If a footstep touches down early, the following transfer will be extended to make up for this
            * time difference and the footstep plan will finish at the expected time.
            */
   public void setExecutionTiming(byte execution_timing)
   {
      execution_timing_ = execution_timing;
   }
   /**
            * When CONTROL_DURATIONS is chosen:
            * The controller will try to achieve the swing_duration and the transfer_duration specified in the message.
            * If a footstep touches down early, the next step will not be affected by this and the whole trajectory might finish earlier than expected.
            * When CONTROL_ABSOLUTE_TIMINGS is chosen:
            * The controller will compute the expected times for swing start and touchdown and attempt to start a footstep at that time.
            * If a footstep touches down early, the following transfer will be extended to make up for this
            * time difference and the footstep plan will finish at the expected time.
            */
   public byte getExecutionTiming()
   {
      return execution_timing_;
   }

   /**
            * The swing_duration is the time a foot is not in ground contact during a step.
            * Each step in a list of footsteps might have a different swing duration.
            * The value specified here is a default value, used if a footstep in this list was created without a swing_duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public void setDefaultSwingDuration(double default_swing_duration)
   {
      default_swing_duration_ = default_swing_duration;
   }
   /**
            * The swing_duration is the time a foot is not in ground contact during a step.
            * Each step in a list of footsteps might have a different swing duration.
            * The value specified here is a default value, used if a footstep in this list was created without a swing_duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public double getDefaultSwingDuration()
   {
      return default_swing_duration_;
   }

   /**
            * The transfer_duration is the time spent with the feet in ground contact before a step.
            * Each step in a list of footsteps might have a different transfer duration.
            * The value specified here is a default value, used if a footstep in this list was created without a transfer-duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public void setDefaultTransferDuration(double default_transfer_duration)
   {
      default_transfer_duration_ = default_transfer_duration;
   }
   /**
            * The transfer_duration is the time spent with the feet in ground contact before a step.
            * Each step in a list of footsteps might have a different transfer duration.
            * The value specified here is a default value, used if a footstep in this list was created without a transfer-duration.
            * When set to zero or a negative value, the controller will its own default value.
            */
   public double getDefaultTransferDuration()
   {
      return default_transfer_duration_;
   }

   /**
            * Specifies the time used to return to a stable standing stance after the execution of the
            * footstep list is finished. If the value is negative the default_transfer_duration will be used,
            * which in turn if not provided indicate the controller to use its own internal default value.
            */
   public void setFinalTransferDuration(double final_transfer_duration)
   {
      final_transfer_duration_ = final_transfer_duration;
   }
   /**
            * Specifies the time used to return to a stable standing stance after the execution of the
            * footstep list is finished. If the value is negative the default_transfer_duration will be used,
            * which in turn if not provided indicate the controller to use its own internal default value.
            */
   public double getFinalTransferDuration()
   {
      return final_transfer_duration_;
   }

   /**
            * If false the controller adjust each footstep height to be at the support sole height.
            */
   public void setTrustHeightOfFootsteps(boolean trust_height_of_footsteps)
   {
      trust_height_of_footsteps_ = trust_height_of_footsteps;
   }
   /**
            * If false the controller adjust each footstep height to be at the support sole height.
            */
   public boolean getTrustHeightOfFootsteps()
   {
      return trust_height_of_footsteps_;
   }

   /**
            * Contains information on whether the robot can automatically adjust its footsteps to retain balance.
            */
   public void setAreFootstepsAdjustable(boolean are_footsteps_adjustable)
   {
      are_footsteps_adjustable_ = are_footsteps_adjustable;
   }
   /**
            * Contains information on whether the robot can automatically adjust its footsteps to retain balance.
            */
   public boolean getAreFootstepsAdjustable()
   {
      return are_footsteps_adjustable_;
   }

   /**
            * If true the controller will adjust upcoming footsteps with the location error of previous steps.
            */
   public void setOffsetFootstepsWithExecutionError(boolean offset_footsteps_with_execution_error)
   {
      offset_footsteps_with_execution_error_ = offset_footsteps_with_execution_error;
   }
   /**
            * If true the controller will adjust upcoming footsteps with the location error of previous steps.
            */
   public boolean getOffsetFootstepsWithExecutionError()
   {
      return offset_footsteps_with_execution_error_;
   }


   /**
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public static Supplier<FootstepDataListMessagePubSubType> getPubSubType()
   {
      return FootstepDataListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepDataListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepDataListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.footstep_data_list_.size() != other.footstep_data_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footstep_data_list_.size(); i++)
         {  if (!this.footstep_data_list_.get(i).epsilonEquals(other.footstep_data_list_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_timing_, other.execution_timing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_swing_duration_, other.default_swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_transfer_duration_, other.default_transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_transfer_duration_, other.final_transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.trust_height_of_footsteps_, other.trust_height_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_footsteps_adjustable_, other.are_footsteps_adjustable_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.offset_footsteps_with_execution_error_, other.offset_footsteps_with_execution_error_, epsilon)) return false;

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepDataListMessage)) return false;

      FootstepDataListMessage otherMyClass = (FootstepDataListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;
      if(this.execution_timing_ != otherMyClass.execution_timing_) return false;

      if(this.default_swing_duration_ != otherMyClass.default_swing_duration_) return false;

      if(this.default_transfer_duration_ != otherMyClass.default_transfer_duration_) return false;

      if(this.final_transfer_duration_ != otherMyClass.final_transfer_duration_) return false;

      if(this.trust_height_of_footsteps_ != otherMyClass.trust_height_of_footsteps_) return false;

      if(this.are_footsteps_adjustable_ != otherMyClass.are_footsteps_adjustable_) return false;

      if(this.offset_footsteps_with_execution_error_ != otherMyClass.offset_footsteps_with_execution_error_) return false;

      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepDataListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);      builder.append(", ");
      builder.append("execution_timing=");
      builder.append(this.execution_timing_);      builder.append(", ");
      builder.append("default_swing_duration=");
      builder.append(this.default_swing_duration_);      builder.append(", ");
      builder.append("default_transfer_duration=");
      builder.append(this.default_transfer_duration_);      builder.append(", ");
      builder.append("final_transfer_duration=");
      builder.append(this.final_transfer_duration_);      builder.append(", ");
      builder.append("trust_height_of_footsteps=");
      builder.append(this.trust_height_of_footsteps_);      builder.append(", ");
      builder.append("are_footsteps_adjustable=");
      builder.append(this.are_footsteps_adjustable_);      builder.append(", ");
      builder.append("offset_footsteps_with_execution_error=");
      builder.append(this.offset_footsteps_with_execution_error_);      builder.append(", ");
      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
