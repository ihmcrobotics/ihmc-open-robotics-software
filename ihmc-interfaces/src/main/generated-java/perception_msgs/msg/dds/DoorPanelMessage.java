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

   public DoorPanelMessage()
   {
      planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
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
      builder.append(this.planar_region_last_update_time_millis_);
      builder.append("}");
      return builder.toString();
   }
}
