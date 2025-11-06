package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BoundingBox2DArray extends Packet<BoundingBox2DArray> implements Settable<BoundingBox2DArray>, EpsilonComparable<BoundingBox2DArray>
{
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.BoundingBox2D>  boxes_;

   public BoundingBox2DArray()
   {
      header_ = new std_msgs.msg.dds.Header();
      boxes_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.BoundingBox2D> (100, new vision_msgs.msg.dds.BoundingBox2DPubSubType());

   }

   public BoundingBox2DArray(BoundingBox2DArray other)
   {
      this();
      set(other);
   }

   public void set(BoundingBox2DArray other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      boxes_.set(other.boxes_);
   }


   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.BoundingBox2D>  getBoxes()
   {
      return boxes_;
   }


   public static Supplier<BoundingBox2DArrayPubSubType> getPubSubType()
   {
      return BoundingBox2DArrayPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BoundingBox2DArrayPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BoundingBox2DArray other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.boxes_.size() != other.boxes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.boxes_.size(); i++)
         {  if (!this.boxes_.get(i).epsilonEquals(other.boxes_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BoundingBox2DArray)) return false;

      BoundingBox2DArray otherMyClass = (BoundingBox2DArray) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.boxes_.equals(otherMyClass.boxes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBox2DArray {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("boxes=");
      builder.append(this.boxes_);
      builder.append("}");
      return builder.toString();
   }
}
