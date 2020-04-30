package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message part of the localization module
       */
public class LocalizationPointMapPacket extends Packet<LocalizationPointMapPacket> implements Settable<LocalizationPointMapPacket>, EpsilonComparable<LocalizationPointMapPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public long timestamp_;

   public us.ihmc.idl.IDLSequence.Float  localization_point_map_;

   public LocalizationPointMapPacket()
   {



      localization_point_map_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");


   }

   public LocalizationPointMapPacket(LocalizationPointMapPacket other)
   {
      this();
      set(other);
   }

   public void set(LocalizationPointMapPacket other)
   {

      sequence_id_ = other.sequence_id_;


      timestamp_ = other.timestamp_;


      localization_point_map_.set(other.localization_point_map_);
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


   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }



   public us.ihmc.idl.IDLSequence.Float  getLocalizationPointMap()
   {
      return localization_point_map_;
   }


   public static Supplier<LocalizationPointMapPacketPubSubType> getPubSubType()
   {
      return LocalizationPointMapPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LocalizationPointMapPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LocalizationPointMapPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.localization_point_map_, other.localization_point_map_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LocalizationPointMapPacket)) return false;

      LocalizationPointMapPacket otherMyClass = (LocalizationPointMapPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.timestamp_ != otherMyClass.timestamp_) return false;


      if (!this.localization_point_map_.equals(otherMyClass.localization_point_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LocalizationPointMapPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");

      builder.append("localization_point_map=");
      builder.append(this.localization_point_map_);
      builder.append("}");
      return builder.toString();
   }
}
