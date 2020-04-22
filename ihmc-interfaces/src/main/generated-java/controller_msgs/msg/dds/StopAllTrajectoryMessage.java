package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Stop the execution of any trajectory being executed.
       */
public class StopAllTrajectoryMessage extends Packet<StopAllTrajectoryMessage> implements Settable<StopAllTrajectoryMessage>, EpsilonComparable<StopAllTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public StopAllTrajectoryMessage()
   {


   }

   public StopAllTrajectoryMessage(StopAllTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(StopAllTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;

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


   public static Supplier<StopAllTrajectoryMessagePubSubType> getPubSubType()
   {
      return StopAllTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StopAllTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StopAllTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StopAllTrajectoryMessage)) return false;

      StopAllTrajectoryMessage otherMyClass = (StopAllTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StopAllTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
