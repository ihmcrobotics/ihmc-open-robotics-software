package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message contains a list of planar regions and a timestamp.
       */
public class TimestampedPlanarRegionsListMessage extends Packet<TimestampedPlanarRegionsListMessage> implements Settable<TimestampedPlanarRegionsListMessage>, EpsilonComparable<TimestampedPlanarRegionsListMessage>
{
   /**
            * Approximate last update time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public long last_updated_seconds_since_epoch_;
   /**
            * Approximate last update time additional nanoseconds
            */
   public long last_updated_additional_nanos_;
   public perception_msgs.msg.dds.PlanarRegionsListMessage planar_regions_;

   public TimestampedPlanarRegionsListMessage()
   {
      planar_regions_ = new perception_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public TimestampedPlanarRegionsListMessage(TimestampedPlanarRegionsListMessage other)
   {
      this();
      set(other);
   }

   public void set(TimestampedPlanarRegionsListMessage other)
   {
      last_updated_seconds_since_epoch_ = other.last_updated_seconds_since_epoch_;

      last_updated_additional_nanos_ = other.last_updated_additional_nanos_;

      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_, planar_regions_);
   }

   /**
            * Approximate last update time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public void setLastUpdatedSecondsSinceEpoch(long last_updated_seconds_since_epoch)
   {
      last_updated_seconds_since_epoch_ = last_updated_seconds_since_epoch;
   }
   /**
            * Approximate last update time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public long getLastUpdatedSecondsSinceEpoch()
   {
      return last_updated_seconds_since_epoch_;
   }

   /**
            * Approximate last update time additional nanoseconds
            */
   public void setLastUpdatedAdditionalNanos(long last_updated_additional_nanos)
   {
      last_updated_additional_nanos_ = last_updated_additional_nanos;
   }
   /**
            * Approximate last update time additional nanoseconds
            */
   public long getLastUpdatedAdditionalNanos()
   {
      return last_updated_additional_nanos_;
   }


   public perception_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegions()
   {
      return planar_regions_;
   }


   public static Supplier<TimestampedPlanarRegionsListMessagePubSubType> getPubSubType()
   {
      return TimestampedPlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TimestampedPlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TimestampedPlanarRegionsListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.last_updated_seconds_since_epoch_, other.last_updated_seconds_since_epoch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.last_updated_additional_nanos_, other.last_updated_additional_nanos_, epsilon)) return false;

      if (!this.planar_regions_.epsilonEquals(other.planar_regions_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TimestampedPlanarRegionsListMessage)) return false;

      TimestampedPlanarRegionsListMessage otherMyClass = (TimestampedPlanarRegionsListMessage) other;

      if(this.last_updated_seconds_since_epoch_ != otherMyClass.last_updated_seconds_since_epoch_) return false;

      if(this.last_updated_additional_nanos_ != otherMyClass.last_updated_additional_nanos_) return false;

      if (!this.planar_regions_.equals(otherMyClass.planar_regions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TimestampedPlanarRegionsListMessage {");
      builder.append("last_updated_seconds_since_epoch=");
      builder.append(this.last_updated_seconds_since_epoch_);      builder.append(", ");
      builder.append("last_updated_additional_nanos=");
      builder.append(this.last_updated_additional_nanos_);      builder.append(", ");
      builder.append("planar_regions=");
      builder.append(this.planar_regions_);
      builder.append("}");
      return builder.toString();
   }
}
