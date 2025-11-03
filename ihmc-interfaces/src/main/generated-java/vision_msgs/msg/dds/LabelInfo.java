package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Provides meta-information about a visual pipeline.
       * 
       * This message serves a similar purpose to sensor_msgs/CameraInfo, but instead
       * of being tied to hardware, it represents information about a specific
       * computer vision pipeline. This information stays constant (or relatively
       * constant) over time, and so it is wasteful to send it with each individual
       * result. By listening to these messages, subscribers will receive
       * the context in which published vision messages are to be interpreted.
       * Each vision pipeline should publish its LabelInfo messages to its own topic,
       * in a manner similar to CameraInfo.
       * This message is meant to allow converting data from vision pipelines that
       * return id based classifications back to human readable string class names.
       */
public class LabelInfo extends Packet<LabelInfo> implements Settable<LabelInfo>, EpsilonComparable<LabelInfo>
{
   /**
            * Used for sequencing
            */
   public std_msgs.msg.dds.Header header_;
   /**
            * An array of uint16 keys and string values containing the association
            * between class identifiers and their names. According to the amount
            * of classes and the datatype used to store their ids internally, the
            * maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to
            * the "UNLABELED" class.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.VisionClass>  class_map_;
   /**
            * The value between 0-1 used as confidence threshold for the inference.
            */
   public float threshold_;

   public LabelInfo()
   {
      header_ = new std_msgs.msg.dds.Header();
      class_map_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.VisionClass> (100, new vision_msgs.msg.dds.VisionClassPubSubType());

   }

   public LabelInfo(LabelInfo other)
   {
      this();
      set(other);
   }

   public void set(LabelInfo other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      class_map_.set(other.class_map_);
      threshold_ = other.threshold_;

   }


   /**
            * Used for sequencing
            */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * An array of uint16 keys and string values containing the association
            * between class identifiers and their names. According to the amount
            * of classes and the datatype used to store their ids internally, the
            * maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to
            * the "UNLABELED" class.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.VisionClass>  getClassMap()
   {
      return class_map_;
   }

   /**
            * The value between 0-1 used as confidence threshold for the inference.
            */
   public void setThreshold(float threshold)
   {
      threshold_ = threshold;
   }
   /**
            * The value between 0-1 used as confidence threshold for the inference.
            */
   public float getThreshold()
   {
      return threshold_;
   }


   public static Supplier<LabelInfoPubSubType> getPubSubType()
   {
      return LabelInfoPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LabelInfoPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LabelInfo other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.class_map_.size() != other.class_map_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.class_map_.size(); i++)
         {  if (!this.class_map_.get(i).epsilonEquals(other.class_map_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.threshold_, other.threshold_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LabelInfo)) return false;

      LabelInfo otherMyClass = (LabelInfo) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.class_map_.equals(otherMyClass.class_map_)) return false;
      if(this.threshold_ != otherMyClass.threshold_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LabelInfo {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("class_map=");
      builder.append(this.class_map_);      builder.append(", ");
      builder.append("threshold=");
      builder.append(this.threshold_);
      builder.append("}");
      return builder.toString();
   }
}
