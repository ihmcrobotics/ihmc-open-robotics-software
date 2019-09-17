package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to switch the control scheme between different control mode.
       */
public class HighLevelStateMessage extends Packet<HighLevelStateMessage> implements Settable<HighLevelStateMessage>, EpsilonComparable<HighLevelStateMessage>
{
   public static final byte DO_NOTHING_BEHAVIOR = (byte) 0;
   public static final byte STAND_PREP_STATE = (byte) 1;
   public static final byte STAND_READY = (byte) 2;
   public static final byte FREEZE_STATE = (byte) 3;
   public static final byte STAND_TRANSITION_STATE = (byte) 4;
   public static final byte WALKING = (byte) 5;
   public static final byte EXIT_WALKING = (byte) 6;
   public static final byte DIAGNOSTICS = (byte) 7;
   public static final byte CALIBRATION = (byte) 8;
   public static final byte CUSTOM1 = (byte) 9;
   public static final byte FALLING_STATE = (byte) 10;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the which state the controller should transition into.
            */
   public byte high_level_controller_name_ = (byte) 255;

   public HighLevelStateMessage()
   {
   }

   public HighLevelStateMessage(HighLevelStateMessage other)
   {
      this();
      set(other);
   }

   public void set(HighLevelStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      high_level_controller_name_ = other.high_level_controller_name_;

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
            * Specifies the which state the controller should transition into.
            */
   public void setHighLevelControllerName(byte high_level_controller_name)
   {
      high_level_controller_name_ = high_level_controller_name;
   }
   /**
            * Specifies the which state the controller should transition into.
            */
   public byte getHighLevelControllerName()
   {
      return high_level_controller_name_;
   }


   public static Supplier<HighLevelStateMessagePubSubType> getPubSubType()
   {
      return HighLevelStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HighLevelStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.high_level_controller_name_, other.high_level_controller_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HighLevelStateMessage)) return false;

      HighLevelStateMessage otherMyClass = (HighLevelStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.high_level_controller_name_ != otherMyClass.high_level_controller_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HighLevelStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("high_level_controller_name=");
      builder.append(this.high_level_controller_name_);
      builder.append("}");
      return builder.toString();
   }
}
