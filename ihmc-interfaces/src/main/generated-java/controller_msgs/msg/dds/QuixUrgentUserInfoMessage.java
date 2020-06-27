package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       * This message is used to notify the crutch display of some important system state that might otherwise be hard to recognize.
       */
public class QuixUrgentUserInfoMessage extends Packet<QuixUrgentUserInfoMessage> implements Settable<QuixUrgentUserInfoMessage>, EpsilonComparable<QuixUrgentUserInfoMessage>
{

   public static final byte EMPTY = (byte) 0;

   public static final byte ESTOP_PRESSED = (byte) 1;

   public static final byte MOTOR_OVER_TEMP = (byte) 2;

   public static final byte LOW_BATTERY = (byte) 3;

   public long sequence_id_;

   public byte urgent_user_info_name_;

   public QuixUrgentUserInfoMessage()
   {



   }

   public QuixUrgentUserInfoMessage(QuixUrgentUserInfoMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixUrgentUserInfoMessage other)
   {

      sequence_id_ = other.sequence_id_;


      urgent_user_info_name_ = other.urgent_user_info_name_;

   }


   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setUrgentUserInfoName(byte urgent_user_info_name)
   {
      urgent_user_info_name_ = urgent_user_info_name;
   }
   public byte getUrgentUserInfoName()
   {
      return urgent_user_info_name_;
   }


   public static Supplier<QuixUrgentUserInfoMessagePubSubType> getPubSubType()
   {
      return QuixUrgentUserInfoMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixUrgentUserInfoMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixUrgentUserInfoMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.urgent_user_info_name_, other.urgent_user_info_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixUrgentUserInfoMessage)) return false;

      QuixUrgentUserInfoMessage otherMyClass = (QuixUrgentUserInfoMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.urgent_user_info_name_ != otherMyClass.urgent_user_info_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixUrgentUserInfoMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("urgent_user_info_name=");
      builder.append(this.urgent_user_info_name_);
      builder.append("}");
      return builder.toString();
   }
}
