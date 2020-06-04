package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       * This message is used to notify the crutch display of the current motion state and allow the crutch to communicate a desired change in state.
       */
public class QuixMotionStateMessage extends Packet<QuixMotionStateMessage> implements Settable<QuixMotionStateMessage>, EpsilonComparable<QuixMotionStateMessage>
{

   public static final byte HOLD_POSITION = (byte) 0;

   public static final byte SIT_DOWN = (byte) 1;

   public static final byte STAND_UP = (byte) 2;

   public static final byte MOVE_TO_FLAT_GROUND = (byte) 3;

   public static final byte FLAT_GROUND_WALKING = (byte) 4;

   public static final byte MOVE_TO_SLOPE = (byte) 5;

   public static final byte SLOPE_WALKING = (byte) 6;

   public static final byte OLD_FLAT_GROUND_WALKING = (byte) 7;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte motion_state_name_ = (byte) 255;

   public QuixMotionStateMessage()
   {



   }

   public QuixMotionStateMessage(QuixMotionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixMotionStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      motion_state_name_ = other.motion_state_name_;

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


   public void setMotionStateName(byte motion_state_name)
   {
      motion_state_name_ = motion_state_name;
   }
   public byte getMotionStateName()
   {
      return motion_state_name_;
   }


   public static Supplier<QuixMotionStateMessagePubSubType> getPubSubType()
   {
      return QuixMotionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixMotionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixMotionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motion_state_name_, other.motion_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixMotionStateMessage)) return false;

      QuixMotionStateMessage otherMyClass = (QuixMotionStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.motion_state_name_ != otherMyClass.motion_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixMotionStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("motion_state_name=");
      builder.append(this.motion_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
