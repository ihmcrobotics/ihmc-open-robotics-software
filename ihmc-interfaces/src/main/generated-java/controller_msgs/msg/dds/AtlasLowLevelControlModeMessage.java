package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message.
       */
public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage> implements Settable<AtlasLowLevelControlModeMessage>, EpsilonComparable<AtlasLowLevelControlModeMessage>
{

   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_STAND_PREP = (byte) 0;

   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_FREEZE = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte requested_atlas_low_level_control_mode_ = (byte) 255;

   public AtlasLowLevelControlModeMessage()
   {



   }

   public AtlasLowLevelControlModeMessage(AtlasLowLevelControlModeMessage other)
   {
      this();
      set(other);
   }

   public void set(AtlasLowLevelControlModeMessage other)
   {

      sequence_id_ = other.sequence_id_;


      requested_atlas_low_level_control_mode_ = other.requested_atlas_low_level_control_mode_;

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


   public void setRequestedAtlasLowLevelControlMode(byte requested_atlas_low_level_control_mode)
   {
      requested_atlas_low_level_control_mode_ = requested_atlas_low_level_control_mode;
   }
   public byte getRequestedAtlasLowLevelControlMode()
   {
      return requested_atlas_low_level_control_mode_;
   }


   public static Supplier<AtlasLowLevelControlModeMessagePubSubType> getPubSubType()
   {
      return AtlasLowLevelControlModeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasLowLevelControlModeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_atlas_low_level_control_mode_, other.requested_atlas_low_level_control_mode_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasLowLevelControlModeMessage)) return false;

      AtlasLowLevelControlModeMessage otherMyClass = (AtlasLowLevelControlModeMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.requested_atlas_low_level_control_mode_ != otherMyClass.requested_atlas_low_level_control_mode_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasLowLevelControlModeMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("requested_atlas_low_level_control_mode=");
      builder.append(this.requested_atlas_low_level_control_mode_);
      builder.append("}");
      return builder.toString();
   }
}
