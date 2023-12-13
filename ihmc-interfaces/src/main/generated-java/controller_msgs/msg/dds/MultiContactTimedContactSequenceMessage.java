package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       * Message for commanding whole-body jointspace trajectory as well as precomputed load status
       */
public class MultiContactTimedContactSequenceMessage extends Packet<MultiContactTimedContactSequenceMessage> implements Settable<MultiContactTimedContactSequenceMessage>, EpsilonComparable<MultiContactTimedContactSequenceMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Timed intervals that left arm is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  left_arm_contact_intervals_;
   /**
            * Timed intervals that right arm is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  right_arm_contact_intervals_;
   /**
            * Timed intervals that left leg is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  left_leg_contact_intervals_;
   /**
            * Timed intervals that right leg is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  right_leg_contact_intervals_;

   public MultiContactTimedContactSequenceMessage()
   {
      left_arm_contact_intervals_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage> (25, new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType());
      right_arm_contact_intervals_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage> (25, new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType());
      left_leg_contact_intervals_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage> (25, new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType());
      right_leg_contact_intervals_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage> (25, new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType());

   }

   public MultiContactTimedContactSequenceMessage(MultiContactTimedContactSequenceMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactTimedContactSequenceMessage other)
   {
      sequence_id_ = other.sequence_id_;

      left_arm_contact_intervals_.set(other.left_arm_contact_intervals_);
      right_arm_contact_intervals_.set(other.right_arm_contact_intervals_);
      left_leg_contact_intervals_.set(other.left_leg_contact_intervals_);
      right_leg_contact_intervals_.set(other.right_leg_contact_intervals_);
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
            * Timed intervals that left arm is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  getLeftArmContactIntervals()
   {
      return left_arm_contact_intervals_;
   }


   /**
            * Timed intervals that right arm is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  getRightArmContactIntervals()
   {
      return right_arm_contact_intervals_;
   }


   /**
            * Timed intervals that left leg is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  getLeftLegContactIntervals()
   {
      return left_leg_contact_intervals_;
   }


   /**
            * Timed intervals that right leg is in contact
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.TimeIntervalMessage>  getRightLegContactIntervals()
   {
      return right_leg_contact_intervals_;
   }


   public static Supplier<MultiContactTimedContactSequenceMessagePubSubType> getPubSubType()
   {
      return MultiContactTimedContactSequenceMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactTimedContactSequenceMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactTimedContactSequenceMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.left_arm_contact_intervals_.size() != other.left_arm_contact_intervals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_arm_contact_intervals_.size(); i++)
         {  if (!this.left_arm_contact_intervals_.get(i).epsilonEquals(other.left_arm_contact_intervals_.get(i), epsilon)) return false; }
      }

      if (this.right_arm_contact_intervals_.size() != other.right_arm_contact_intervals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_arm_contact_intervals_.size(); i++)
         {  if (!this.right_arm_contact_intervals_.get(i).epsilonEquals(other.right_arm_contact_intervals_.get(i), epsilon)) return false; }
      }

      if (this.left_leg_contact_intervals_.size() != other.left_leg_contact_intervals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_leg_contact_intervals_.size(); i++)
         {  if (!this.left_leg_contact_intervals_.get(i).epsilonEquals(other.left_leg_contact_intervals_.get(i), epsilon)) return false; }
      }

      if (this.right_leg_contact_intervals_.size() != other.right_leg_contact_intervals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_leg_contact_intervals_.size(); i++)
         {  if (!this.right_leg_contact_intervals_.get(i).epsilonEquals(other.right_leg_contact_intervals_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactTimedContactSequenceMessage)) return false;

      MultiContactTimedContactSequenceMessage otherMyClass = (MultiContactTimedContactSequenceMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.left_arm_contact_intervals_.equals(otherMyClass.left_arm_contact_intervals_)) return false;
      if (!this.right_arm_contact_intervals_.equals(otherMyClass.right_arm_contact_intervals_)) return false;
      if (!this.left_leg_contact_intervals_.equals(otherMyClass.left_leg_contact_intervals_)) return false;
      if (!this.right_leg_contact_intervals_.equals(otherMyClass.right_leg_contact_intervals_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactTimedContactSequenceMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("left_arm_contact_intervals=");
      builder.append(this.left_arm_contact_intervals_);      builder.append(", ");
      builder.append("right_arm_contact_intervals=");
      builder.append(this.right_arm_contact_intervals_);      builder.append(", ");
      builder.append("left_leg_contact_intervals=");
      builder.append(this.left_leg_contact_intervals_);      builder.append(", ");
      builder.append("right_leg_contact_intervals=");
      builder.append(this.right_leg_contact_intervals_);
      builder.append("}");
      return builder.toString();
   }
}
