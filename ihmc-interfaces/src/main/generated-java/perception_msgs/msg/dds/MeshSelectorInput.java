package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MeshSelectorInput extends Packet<MeshSelectorInput> implements Settable<MeshSelectorInput>, EpsilonComparable<MeshSelectorInput>
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
            * RGB image corresponding to the detection
            */
   public sensor_msgs.msg.dds.Image rgb_image_;
   /**
            * Binary segmentation mask for the detected object
            */
   public sensor_msgs.msg.dds.Image mask_image_;

   public MeshSelectorInput()
   {
      category_ = new java.lang.StringBuilder(255);
      rgb_image_ = new sensor_msgs.msg.dds.Image();
      mask_image_ = new sensor_msgs.msg.dds.Image();
   }

   public MeshSelectorInput(MeshSelectorInput other)
   {
      this();
      set(other);
   }

   public void set(MeshSelectorInput other)
   {
      category_.setLength(0);
      category_.append(other.category_);

      track_id_ = other.track_id_;

      sensor_msgs.msg.dds.ImagePubSubType.staticCopy(other.rgb_image_, rgb_image_);
      sensor_msgs.msg.dds.ImagePubSubType.staticCopy(other.mask_image_, mask_image_);
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
            * RGB image corresponding to the detection
            */
   public sensor_msgs.msg.dds.Image getRgbImage()
   {
      return rgb_image_;
   }


   /**
            * Binary segmentation mask for the detected object
            */
   public sensor_msgs.msg.dds.Image getMaskImage()
   {
      return mask_image_;
   }


   public static Supplier<MeshSelectorInputPubSubType> getPubSubType()
   {
      return MeshSelectorInputPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MeshSelectorInputPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MeshSelectorInput other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.category_, other.category_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.track_id_, other.track_id_, epsilon)) return false;

      if (!this.rgb_image_.epsilonEquals(other.rgb_image_, epsilon)) return false;
      if (!this.mask_image_.epsilonEquals(other.mask_image_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MeshSelectorInput)) return false;

      MeshSelectorInput otherMyClass = (MeshSelectorInput) other;

      if (!us.ihmc.idl.IDLTools.equals(this.category_, otherMyClass.category_)) return false;

      if(this.track_id_ != otherMyClass.track_id_) return false;

      if (!this.rgb_image_.equals(otherMyClass.rgb_image_)) return false;
      if (!this.mask_image_.equals(otherMyClass.mask_image_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MeshSelectorInput {");
      builder.append("category=");
      builder.append(this.category_);      builder.append(", ");
      builder.append("track_id=");
      builder.append(this.track_id_);      builder.append(", ");
      builder.append("rgb_image=");
      builder.append(this.rgb_image_);      builder.append(", ");
      builder.append("mask_image=");
      builder.append(this.mask_image_);
      builder.append("}");
      return builder.toString();
   }
}
