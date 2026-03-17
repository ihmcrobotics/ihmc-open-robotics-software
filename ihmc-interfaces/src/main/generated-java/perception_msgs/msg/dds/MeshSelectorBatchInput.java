package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MeshSelectorBatchInput extends Packet<MeshSelectorBatchInput> implements Settable<MeshSelectorBatchInput>, EpsilonComparable<MeshSelectorBatchInput>
{
   /**
            * Frame ID of the image
            */
   public long frame_id_;
   /**
            * Array of Mesh Selector Batch Items
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.MeshSelectorBatchItem>  items_;

   public MeshSelectorBatchInput()
   {
      items_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.MeshSelectorBatchItem> (100, new perception_msgs.msg.dds.MeshSelectorBatchItemPubSubType());

   }

   public MeshSelectorBatchInput(MeshSelectorBatchInput other)
   {
      this();
      set(other);
   }

   public void set(MeshSelectorBatchInput other)
   {
      frame_id_ = other.frame_id_;

      items_.set(other.items_);
   }

   /**
            * Frame ID of the image
            */
   public void setFrameId(long frame_id)
   {
      frame_id_ = frame_id;
   }
   /**
            * Frame ID of the image
            */
   public long getFrameId()
   {
      return frame_id_;
   }


   /**
            * Array of Mesh Selector Batch Items
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.MeshSelectorBatchItem>  getItems()
   {
      return items_;
   }


   public static Supplier<MeshSelectorBatchInputPubSubType> getPubSubType()
   {
      return MeshSelectorBatchInputPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MeshSelectorBatchInputPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MeshSelectorBatchInput other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frame_id_, other.frame_id_, epsilon)) return false;

      if (this.items_.size() != other.items_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.items_.size(); i++)
         {  if (!this.items_.get(i).epsilonEquals(other.items_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MeshSelectorBatchInput)) return false;

      MeshSelectorBatchInput otherMyClass = (MeshSelectorBatchInput) other;

      if(this.frame_id_ != otherMyClass.frame_id_) return false;

      if (!this.items_.equals(otherMyClass.items_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MeshSelectorBatchInput {");
      builder.append("frame_id=");
      builder.append(this.frame_id_);      builder.append(", ");
      builder.append("items=");
      builder.append(this.items_);
      builder.append("}");
      return builder.toString();
   }
}
