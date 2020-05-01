package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to abort walking, forcing the robot to switch back to double support and clear the footstep list.
       */
public class AbortWalkingMessage extends Packet<AbortWalkingMessage> implements Settable<AbortWalkingMessage>, EpsilonComparable<AbortWalkingMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public AbortWalkingMessage()
   {


   }

   public AbortWalkingMessage(AbortWalkingMessage other)
   {
      this();
      set(other);
   }

   public void set(AbortWalkingMessage other)
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


   public static Supplier<AbortWalkingMessagePubSubType> getPubSubType()
   {
      return AbortWalkingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbortWalkingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbortWalkingMessage other, double epsilon)
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
      if(!(other instanceof AbortWalkingMessage)) return false;

      AbortWalkingMessage otherMyClass = (AbortWalkingMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbortWalkingMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
