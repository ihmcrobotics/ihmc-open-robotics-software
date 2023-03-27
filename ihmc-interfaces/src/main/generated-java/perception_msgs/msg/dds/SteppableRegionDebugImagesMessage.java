package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SteppableRegionDebugImagesMessage extends Packet<SteppableRegionDebugImagesMessage> implements Settable<SteppableRegionDebugImagesMessage>, EpsilonComparable<SteppableRegionDebugImagesMessage>
{
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage>  steppability_images_;
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage>  region_images_;

   public SteppableRegionDebugImagesMessage()
   {
      steppability_images_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage> (10, new perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType());
      region_images_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage> (10, new perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType());

   }

   public SteppableRegionDebugImagesMessage(SteppableRegionDebugImagesMessage other)
   {
      this();
      set(other);
   }

   public void set(SteppableRegionDebugImagesMessage other)
   {
      steppability_images_.set(other.steppability_images_);
      region_images_.set(other.region_images_);
   }


   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage>  getSteppabilityImages()
   {
      return steppability_images_;
   }


   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SteppableRegionDebugImageMessage>  getRegionImages()
   {
      return region_images_;
   }


   public static Supplier<SteppableRegionDebugImagesMessagePubSubType> getPubSubType()
   {
      return SteppableRegionDebugImagesMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SteppableRegionDebugImagesMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SteppableRegionDebugImagesMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.steppability_images_.size() != other.steppability_images_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.steppability_images_.size(); i++)
         {  if (!this.steppability_images_.get(i).epsilonEquals(other.steppability_images_.get(i), epsilon)) return false; }
      }

      if (this.region_images_.size() != other.region_images_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_images_.size(); i++)
         {  if (!this.region_images_.get(i).epsilonEquals(other.region_images_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SteppableRegionDebugImagesMessage)) return false;

      SteppableRegionDebugImagesMessage otherMyClass = (SteppableRegionDebugImagesMessage) other;

      if (!this.steppability_images_.equals(otherMyClass.steppability_images_)) return false;
      if (!this.region_images_.equals(otherMyClass.region_images_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SteppableRegionDebugImagesMessage {");
      builder.append("steppability_images=");
      builder.append(this.steppability_images_);      builder.append(", ");
      builder.append("region_images=");
      builder.append(this.region_images_);
      builder.append("}");
      return builder.toString();
   }
}
