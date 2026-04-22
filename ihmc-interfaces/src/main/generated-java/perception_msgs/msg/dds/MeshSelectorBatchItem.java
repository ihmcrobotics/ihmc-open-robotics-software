package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MeshSelectorBatchItem extends Packet<MeshSelectorBatchItem> implements Settable<MeshSelectorBatchItem>, EpsilonComparable<MeshSelectorBatchItem>
{
   /**
            * Category / class label from the detector
            */
   public java.lang.StringBuilder category_;
   /**
            * Tracking ID assigned by the detection tracker
            */
   public int track_id_;
   /**
            * Segmented RGB image corresponding to the detection
            */
   public sensor_msgs.msg.dds.Image segmented_rgb_;

   public MeshSelectorBatchItem()
   {
      category_ = new java.lang.StringBuilder(255);
      segmented_rgb_ = new sensor_msgs.msg.dds.Image();
   }

   public MeshSelectorBatchItem(MeshSelectorBatchItem other)
   {
      this();
      set(other);
   }

   public void set(MeshSelectorBatchItem other)
   {
      category_.setLength(0);
      category_.append(other.category_);

      track_id_ = other.track_id_;

      sensor_msgs.msg.dds.ImagePubSubType.staticCopy(other.segmented_rgb_, segmented_rgb_);
   }

   /**
            * Category / class label from the detector
            */
   public void setCategory(java.lang.String category)
   {
      category_.setLength(0);
      category_.append(category);
   }

   /**
            * Category / class label from the detector
            */
   public java.lang.String getCategoryAsString()
   {
      return getCategory().toString();
   }
   /**
            * Category / class label from the detector
            */
   public java.lang.StringBuilder getCategory()
   {
      return category_;
   }

   /**
            * Tracking ID assigned by the detection tracker
            */
   public void setTrackId(int track_id)
   {
      track_id_ = track_id;
   }
   /**
            * Tracking ID assigned by the detection tracker
            */
   public int getTrackId()
   {
      return track_id_;
   }


   /**
            * Segmented RGB image corresponding to the detection
            */
   public sensor_msgs.msg.dds.Image getSegmentedRgb()
   {
      return segmented_rgb_;
   }


   public static Supplier<MeshSelectorBatchItemPubSubType> getPubSubType()
   {
      return MeshSelectorBatchItemPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MeshSelectorBatchItemPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MeshSelectorBatchItem other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.category_, other.category_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.track_id_, other.track_id_, epsilon)) return false;

      if (!this.segmented_rgb_.epsilonEquals(other.segmented_rgb_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MeshSelectorBatchItem)) return false;

      MeshSelectorBatchItem otherMyClass = (MeshSelectorBatchItem) other;

      if (!us.ihmc.idl.IDLTools.equals(this.category_, otherMyClass.category_)) return false;

      if(this.track_id_ != otherMyClass.track_id_) return false;

      if (!this.segmented_rgb_.equals(otherMyClass.segmented_rgb_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MeshSelectorBatchItem {");
      builder.append("category=");
      builder.append(this.category_);      builder.append(", ");
      builder.append("track_id=");
      builder.append(this.track_id_);      builder.append(", ");
      builder.append("segmented_rgb=");
      builder.append(this.segmented_rgb_);
      builder.append("}");
      return builder.toString();
   }
}
