package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Defines a 2D detection result.
       * 
       * This is similar to a 2D classification, but includes position information,
       * allowing a classification result for a specific crop or image point to
       * to be located in the larger image.
       */
public class Detection2D extends Packet<Detection2D> implements Settable<Detection2D>, EpsilonComparable<Detection2D>
{
   public std_msgs.msg.dds.Header header_;
   /**
            * Class probabilities
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesisWithPose>  results_;
   /**
            * 2D bounding box surrounding the object.
            */
   public vision_msgs.msg.dds.BoundingBox2D bbox_;
   /**
            * ID used for consistency across multiple detection messages. Detections
            * of the same object in different detection messages should have the same id.
            * This field may be empty.
            */
   public java.lang.StringBuilder id_;

   public Detection2D()
   {
      header_ = new std_msgs.msg.dds.Header();
      results_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesisWithPose> (100, new vision_msgs.msg.dds.ObjectHypothesisWithPosePubSubType());
      bbox_ = new vision_msgs.msg.dds.BoundingBox2D();
      id_ = new java.lang.StringBuilder(255);

   }

   public Detection2D(Detection2D other)
   {
      this();
      set(other);
   }

   public void set(Detection2D other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      results_.set(other.results_);
      vision_msgs.msg.dds.BoundingBox2DPubSubType.staticCopy(other.bbox_, bbox_);
      id_.setLength(0);
      id_.append(other.id_);

   }


   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * Class probabilities
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesisWithPose>  getResults()
   {
      return results_;
   }


   /**
            * 2D bounding box surrounding the object.
            */
   public vision_msgs.msg.dds.BoundingBox2D getBbox()
   {
      return bbox_;
   }

   /**
            * ID used for consistency across multiple detection messages. Detections
            * of the same object in different detection messages should have the same id.
            * This field may be empty.
            */
   public void setId(java.lang.String id)
   {
      id_.setLength(0);
      id_.append(id);
   }

   /**
            * ID used for consistency across multiple detection messages. Detections
            * of the same object in different detection messages should have the same id.
            * This field may be empty.
            */
   public java.lang.String getIdAsString()
   {
      return getId().toString();
   }
   /**
            * ID used for consistency across multiple detection messages. Detections
            * of the same object in different detection messages should have the same id.
            * This field may be empty.
            */
   public java.lang.StringBuilder getId()
   {
      return id_;
   }


   public static Supplier<Detection2DPubSubType> getPubSubType()
   {
      return Detection2DPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Detection2DPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Detection2D other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.results_.size() != other.results_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.results_.size(); i++)
         {  if (!this.results_.get(i).epsilonEquals(other.results_.get(i), epsilon)) return false; }
      }

      if (!this.bbox_.epsilonEquals(other.bbox_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.id_, other.id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Detection2D)) return false;

      Detection2D otherMyClass = (Detection2D) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.results_.equals(otherMyClass.results_)) return false;
      if (!this.bbox_.equals(otherMyClass.bbox_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.id_, otherMyClass.id_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Detection2D {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("results=");
      builder.append(this.results_);      builder.append(", ");
      builder.append("bbox=");
      builder.append(this.bbox_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);
      builder.append("}");
      return builder.toString();
   }
}
