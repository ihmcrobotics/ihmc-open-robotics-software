package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message part of the localization module
       */
public class LocalizationPacket extends Packet<LocalizationPacket> implements Settable<LocalizationPacket>, EpsilonComparable<LocalizationPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean reset_;

   public boolean toggle_;

   public LocalizationPacket()
   {




   }

   public LocalizationPacket(LocalizationPacket other)
   {
      this();
      set(other);
   }

   public void set(LocalizationPacket other)
   {

      sequence_id_ = other.sequence_id_;


      reset_ = other.reset_;


      toggle_ = other.toggle_;

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


   public void setReset(boolean reset)
   {
      reset_ = reset;
   }
   public boolean getReset()
   {
      return reset_;
   }


   public void setToggle(boolean toggle)
   {
      toggle_ = toggle;
   }
   public boolean getToggle()
   {
      return toggle_;
   }


   public static Supplier<LocalizationPacketPubSubType> getPubSubType()
   {
      return LocalizationPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LocalizationPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LocalizationPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reset_, other.reset_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.toggle_, other.toggle_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LocalizationPacket)) return false;

      LocalizationPacket otherMyClass = (LocalizationPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.reset_ != otherMyClass.reset_) return false;


      if(this.toggle_ != otherMyClass.toggle_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LocalizationPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("reset=");
      builder.append(this.reset_);      builder.append(", ");

      builder.append("toggle=");
      builder.append(this.toggle_);
      builder.append("}");
      return builder.toString();
   }
}
