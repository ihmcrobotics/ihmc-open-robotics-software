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
public class ExoStepDataListMessage extends Packet<ExoStepDataListMessage> implements Settable<ExoStepDataListMessage>, EpsilonComparable<ExoStepDataListMessage>
{

   public static final byte STEP_TYPE_FLAT_WALKING = (byte) 0;

   public static final byte STEP_TYPE_STAIRS = (byte) 1;

   public static final byte STEP_TYPE_STEPPING_STONES = (byte) 2;

   public static final byte STEP_TYPE_SLOPES = (byte) 3;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ExoStepDataMessage>  step_data_list_;

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
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;

   public byte step_type_;

   public ExoStepDataListMessage()
   {


      step_data_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ExoStepDataMessage> (50, new controller_msgs.msg.dds.ExoStepDataMessagePubSubType());



      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();


   }

   public ExoStepDataListMessage(ExoStepDataListMessage other)
   {
      this();
      set(other);
   }

   public void set(ExoStepDataListMessage other)
   {

      sequence_id_ = other.sequence_id_;


      step_data_list_.set(other.step_data_list_);

      default_swing_duration_ = other.default_swing_duration_;


      default_transfer_duration_ = other.default_transfer_duration_;


      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);

      step_type_ = other.step_type_;

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
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ExoStepDataMessage>  getStepDataList()
   {
      return step_data_list_;
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
            * Properties for queueing footstep lists.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public void setStepType(byte step_type)
   {
      step_type_ = step_type;
   }
   public byte getStepType()
   {
      return step_type_;
   }


   public static Supplier<ExoStepDataListMessagePubSubType> getPubSubType()
   {
      return ExoStepDataListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExoStepDataListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExoStepDataListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.step_data_list_.size() != other.step_data_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.step_data_list_.size(); i++)
         {  if (!this.step_data_list_.get(i).epsilonEquals(other.step_data_list_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_swing_duration_, other.default_swing_duration_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_transfer_duration_, other.default_transfer_duration_, epsilon)) return false;


      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_type_, other.step_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExoStepDataListMessage)) return false;

      ExoStepDataListMessage otherMyClass = (ExoStepDataListMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.step_data_list_.equals(otherMyClass.step_data_list_)) return false;

      if(this.default_swing_duration_ != otherMyClass.default_swing_duration_) return false;


      if(this.default_transfer_duration_ != otherMyClass.default_transfer_duration_) return false;


      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      if(this.step_type_ != otherMyClass.step_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoStepDataListMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("step_data_list=");
      builder.append(this.step_data_list_);      builder.append(", ");

      builder.append("default_swing_duration=");
      builder.append(this.default_swing_duration_);      builder.append(", ");

      builder.append("default_transfer_duration=");
      builder.append(this.default_transfer_duration_);      builder.append(", ");

      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);      builder.append(", ");

      builder.append("step_type=");
      builder.append(this.step_type_);
      builder.append("}");
      return builder.toString();
   }
}
