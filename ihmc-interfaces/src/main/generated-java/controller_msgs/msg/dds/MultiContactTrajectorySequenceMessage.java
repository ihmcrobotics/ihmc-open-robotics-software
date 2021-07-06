package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       */
public class MultiContactTrajectorySequenceMessage extends Packet<MultiContactTrajectorySequenceMessage> implements Settable<MultiContactTrajectorySequenceMessage>, EpsilonComparable<MultiContactTrajectorySequenceMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Sequence of trajectories to be executed in succession
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.MultiContactTrajectoryMessage>  trajectory_sequence_;

   public MultiContactTrajectorySequenceMessage()
   {
      trajectory_sequence_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.MultiContactTrajectoryMessage> (50, new controller_msgs.msg.dds.MultiContactTrajectoryMessagePubSubType());

   }

   public MultiContactTrajectorySequenceMessage(MultiContactTrajectorySequenceMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactTrajectorySequenceMessage other)
   {
      sequence_id_ = other.sequence_id_;

      trajectory_sequence_.set(other.trajectory_sequence_);
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
            * Sequence of trajectories to be executed in succession
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.MultiContactTrajectoryMessage>  getTrajectorySequence()
   {
      return trajectory_sequence_;
   }


   public static Supplier<MultiContactTrajectorySequenceMessagePubSubType> getPubSubType()
   {
      return MultiContactTrajectorySequenceMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactTrajectorySequenceMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactTrajectorySequenceMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.trajectory_sequence_.size() != other.trajectory_sequence_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.trajectory_sequence_.size(); i++)
         {  if (!this.trajectory_sequence_.get(i).epsilonEquals(other.trajectory_sequence_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactTrajectorySequenceMessage)) return false;

      MultiContactTrajectorySequenceMessage otherMyClass = (MultiContactTrajectorySequenceMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.trajectory_sequence_.equals(otherMyClass.trajectory_sequence_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactTrajectorySequenceMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("trajectory_sequence=");
      builder.append(this.trajectory_sequence_);
      builder.append("}");
      return builder.toString();
   }
}
