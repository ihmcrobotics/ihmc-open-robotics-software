package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class QuixStartupStateMessage extends Packet<QuixStartupStateMessage> implements Settable<QuixStartupStateMessage>, EpsilonComparable<QuixStartupStateMessage>
{

   public static final byte INITIALIZING = (byte) 0;

   public static final byte WAIT_FOR_USER = (byte) 1;

   public static final byte RECOVERABLE_FAULT = (byte) 2;

   public static final byte FATAL_FAULT = (byte) 3;

   public static final byte LIMP = (byte) 4;

   public static final byte WIGGLE = (byte) 5;

   public static final byte HOLD_ALL_READY = (byte) 6;

   public static final byte HOLD_SOME_LOCKED = (byte) 7;

   public static final byte CALL_BEHAVIOR = (byte) 8;

   public long sequence_id_;

   public byte startup_state_name_ = (byte) 255;

   public QuixStartupStateMessage()
   {



   }

   public QuixStartupStateMessage(QuixStartupStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixStartupStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      startup_state_name_ = other.startup_state_name_;

   }


   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setStartupStateName(byte startup_state_name)
   {
      startup_state_name_ = startup_state_name;
   }
   public byte getStartupStateName()
   {
      return startup_state_name_;
   }


   public static Supplier<QuixStartupStateMessagePubSubType> getPubSubType()
   {
      return QuixStartupStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixStartupStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixStartupStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.startup_state_name_, other.startup_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixStartupStateMessage)) return false;

      QuixStartupStateMessage otherMyClass = (QuixStartupStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.startup_state_name_ != otherMyClass.startup_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixStartupStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("startup_state_name=");
      builder.append(this.startup_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
