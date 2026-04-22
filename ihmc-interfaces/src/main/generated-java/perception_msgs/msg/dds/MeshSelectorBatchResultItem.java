package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MeshSelectorBatchResultItem extends Packet<MeshSelectorBatchResultItem> implements Settable<MeshSelectorBatchResultItem>, EpsilonComparable<MeshSelectorBatchResultItem>
{
   /**
            * Object category from the detector
            */
   public java.lang.StringBuilder category_;
   /**
            * Tracking ID of the object instance
            */
   public int track_id_;
   /**
            * Selected CAD mesh instance name
            */
   public java.lang.StringBuilder selected_instance_;
   /**
            * Similarity score between query descriptor and selected mesh descriptor
            */
   public float score_;

   public MeshSelectorBatchResultItem()
   {
      category_ = new java.lang.StringBuilder(255);
      selected_instance_ = new java.lang.StringBuilder(255);
   }

   public MeshSelectorBatchResultItem(MeshSelectorBatchResultItem other)
   {
      this();
      set(other);
   }

   public void set(MeshSelectorBatchResultItem other)
   {
      category_.setLength(0);
      category_.append(other.category_);

      track_id_ = other.track_id_;

      selected_instance_.setLength(0);
      selected_instance_.append(other.selected_instance_);

      score_ = other.score_;

   }

   /**
            * Object category from the detector
            */
   public void setCategory(java.lang.String category)
   {
      category_.setLength(0);
      category_.append(category);
   }

   /**
            * Object category from the detector
            */
   public java.lang.String getCategoryAsString()
   {
      return getCategory().toString();
   }
   /**
            * Object category from the detector
            */
   public java.lang.StringBuilder getCategory()
   {
      return category_;
   }

   /**
            * Tracking ID of the object instance
            */
   public void setTrackId(int track_id)
   {
      track_id_ = track_id;
   }
   /**
            * Tracking ID of the object instance
            */
   public int getTrackId()
   {
      return track_id_;
   }

   /**
            * Selected CAD mesh instance name
            */
   public void setSelectedInstance(java.lang.String selected_instance)
   {
      selected_instance_.setLength(0);
      selected_instance_.append(selected_instance);
   }

   /**
            * Selected CAD mesh instance name
            */
   public java.lang.String getSelectedInstanceAsString()
   {
      return getSelectedInstance().toString();
   }
   /**
            * Selected CAD mesh instance name
            */
   public java.lang.StringBuilder getSelectedInstance()
   {
      return selected_instance_;
   }

   /**
            * Similarity score between query descriptor and selected mesh descriptor
            */
   public void setScore(float score)
   {
      score_ = score;
   }
   /**
            * Similarity score between query descriptor and selected mesh descriptor
            */
   public float getScore()
   {
      return score_;
   }


   public static Supplier<MeshSelectorBatchResultItemPubSubType> getPubSubType()
   {
      return MeshSelectorBatchResultItemPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MeshSelectorBatchResultItemPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MeshSelectorBatchResultItem other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.category_, other.category_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.track_id_, other.track_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.selected_instance_, other.selected_instance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.score_, other.score_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MeshSelectorBatchResultItem)) return false;

      MeshSelectorBatchResultItem otherMyClass = (MeshSelectorBatchResultItem) other;

      if (!us.ihmc.idl.IDLTools.equals(this.category_, otherMyClass.category_)) return false;

      if(this.track_id_ != otherMyClass.track_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.selected_instance_, otherMyClass.selected_instance_)) return false;

      if(this.score_ != otherMyClass.score_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MeshSelectorBatchResultItem {");
      builder.append("category=");
      builder.append(this.category_);      builder.append(", ");
      builder.append("track_id=");
      builder.append(this.track_id_);      builder.append(", ");
      builder.append("selected_instance=");
      builder.append(this.selected_instance_);      builder.append(", ");
      builder.append("score=");
      builder.append(this.score_);
      builder.append("}");
      return builder.toString();
   }
}
