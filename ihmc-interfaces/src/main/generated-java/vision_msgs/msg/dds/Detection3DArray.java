package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A list of 3D detections, for a multi-object 3D detector.
       */
public class Detection3DArray extends Packet<Detection3DArray> implements Settable<Detection3DArray>, EpsilonComparable<Detection3DArray>
{
   public std_msgs.msg.dds.Header header_;
   /**
            * A list of the detected proposals. A multi-proposal detector might generate
            * this list with many candidate detections generated from a single input.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Detection3D>  detections_;

   public Detection3DArray()
   {
      header_ = new std_msgs.msg.dds.Header();
      detections_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Detection3D> (100, new vision_msgs.msg.dds.Detection3DPubSubType());

   }

   public Detection3DArray(Detection3DArray other)
   {
      this();
      set(other);
   }

   public void set(Detection3DArray other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      detections_.set(other.detections_);
   }


   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * A list of the detected proposals. A multi-proposal detector might generate
            * this list with many candidate detections generated from a single input.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Detection3D>  getDetections()
   {
      return detections_;
   }


   public static Supplier<Detection3DArrayPubSubType> getPubSubType()
   {
      return Detection3DArrayPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Detection3DArrayPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Detection3DArray other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.detections_.size() != other.detections_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detections_.size(); i++)
         {  if (!this.detections_.get(i).epsilonEquals(other.detections_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Detection3DArray)) return false;

      Detection3DArray otherMyClass = (Detection3DArray) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.detections_.equals(otherMyClass.detections_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Detection3DArray {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("detections=");
      builder.append(this.detections_);
      builder.append("}");
      return builder.toString();
   }
}
