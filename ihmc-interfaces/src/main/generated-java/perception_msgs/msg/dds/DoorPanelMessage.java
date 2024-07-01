package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorPanelMessage extends Packet<DoorPanelMessage> implements Settable<DoorPanelMessage>, EpsilonComparable<DoorPanelMessage>
{
   /**
            * The closest fit planar region that represents the door panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage planar_region_;
   /**
            * The time in milliseconds the planar region was last updated
            */
   public long planar_region_last_update_time_millis_;
   /**
            * ID of the persistent detection associated with this panel. May be null.
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage persistent_detection_id_;

   public DoorPanelMessage()
   {
      planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
      persistent_detection_id_ = new ihmc_common_msgs.msg.dds.UUIDMessage();
   }

   public DoorPanelMessage(DoorPanelMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorPanelMessage other)
   {
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.planar_region_, planar_region_);
      planar_region_last_update_time_millis_ = other.planar_region_last_update_time_millis_;

      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.staticCopy(other.persistent_detection_id_, persistent_detection_id_);
   }


   /**
            * The closest fit planar region that represents the door panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage getPlanarRegion()
   {
      return planar_region_;
   }

   /**
            * The time in milliseconds the planar region was last updated
            */
   public void setPlanarRegionLastUpdateTimeMillis(long planar_region_last_update_time_millis)
   {
      planar_region_last_update_time_millis_ = planar_region_last_update_time_millis;
   }
   /**
            * The time in milliseconds the planar region was last updated
            */
   public long getPlanarRegionLastUpdateTimeMillis()
   {
      return planar_region_last_update_time_millis_;
   }


   /**
            * ID of the persistent detection associated with this panel. May be null.
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage getPersistentDetectionId()
   {
      return persistent_detection_id_;
   }


   public static Supplier<DoorPanelMessagePubSubType> getPubSubType()
   {
      return DoorPanelMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorPanelMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorPanelMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.planar_region_.epsilonEquals(other.planar_region_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_region_last_update_time_millis_, other.planar_region_last_update_time_millis_, epsilon)) return false;

      if (!this.persistent_detection_id_.epsilonEquals(other.persistent_detection_id_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorPanelMessage)) return false;

      DoorPanelMessage otherMyClass = (DoorPanelMessage) other;

      if (!this.planar_region_.equals(otherMyClass.planar_region_)) return false;
      if(this.planar_region_last_update_time_millis_ != otherMyClass.planar_region_last_update_time_millis_) return false;

      if (!this.persistent_detection_id_.equals(otherMyClass.persistent_detection_id_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorPanelMessage {");
      builder.append("planar_region=");
      builder.append(this.planar_region_);      builder.append(", ");
      builder.append("planar_region_last_update_time_millis=");
      builder.append(this.planar_region_last_update_time_millis_);      builder.append(", ");
      builder.append("persistent_detection_id=");
      builder.append(this.persistent_detection_id_);
      builder.append("}");
      return builder.toString();
   }
}
