package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BipedTimedStepListMessage extends Packet<BipedTimedStepListMessage> implements Settable<BipedTimedStepListMessage>, EpsilonComparable<BipedTimedStepListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.BipedTimedStepMessage>  timed_step_list_;

   public BipedTimedStepListMessage()
   {
      timed_step_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.BipedTimedStepMessage> (50, new controller_msgs.msg.dds.BipedTimedStepMessagePubSubType());

   }

   public BipedTimedStepListMessage(BipedTimedStepListMessage other)
   {
      this();
      set(other);
   }

   public void set(BipedTimedStepListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      timed_step_list_.set(other.timed_step_list_);
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
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.BipedTimedStepMessage>  getTimedStepList()
   {
      return timed_step_list_;
   }


   public static Supplier<BipedTimedStepListMessagePubSubType> getPubSubType()
   {
      return BipedTimedStepListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BipedTimedStepListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BipedTimedStepListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.timed_step_list_.size() != other.timed_step_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.timed_step_list_.size(); i++)
         {  if (!this.timed_step_list_.get(i).epsilonEquals(other.timed_step_list_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BipedTimedStepListMessage)) return false;

      BipedTimedStepListMessage otherMyClass = (BipedTimedStepListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.timed_step_list_.equals(otherMyClass.timed_step_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BipedTimedStepListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("timed_step_list=");
      builder.append(this.timed_step_list_);
      builder.append("}");
      return builder.toString();
   }
}
