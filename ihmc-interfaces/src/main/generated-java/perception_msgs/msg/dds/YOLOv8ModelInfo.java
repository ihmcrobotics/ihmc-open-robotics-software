package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Information about a YOLO model
       */
public class YOLOv8ModelInfo extends Packet<YOLOv8ModelInfo> implements Settable<YOLOv8ModelInfo>, EpsilonComparable<YOLOv8ModelInfo>
{
   /**
            * Model's name
            */
   public java.lang.StringBuilder model_name_;
   /**
            * Names of object classes the model can detect
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  detectable_object_classes_;

   public YOLOv8ModelInfo()
   {
      model_name_ = new java.lang.StringBuilder(255);
      detectable_object_classes_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (96, "type_d");
   }

   public YOLOv8ModelInfo(YOLOv8ModelInfo other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ModelInfo other)
   {
      model_name_.setLength(0);
      model_name_.append(other.model_name_);

      detectable_object_classes_.set(other.detectable_object_classes_);
   }

   /**
            * Model's name
            */
   public void setModelName(java.lang.String model_name)
   {
      model_name_.setLength(0);
      model_name_.append(model_name);
   }

   /**
            * Model's name
            */
   public java.lang.String getModelNameAsString()
   {
      return getModelName().toString();
   }
   /**
            * Model's name
            */
   public java.lang.StringBuilder getModelName()
   {
      return model_name_;
   }


   /**
            * Names of object classes the model can detect
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getDetectableObjectClasses()
   {
      return detectable_object_classes_;
   }


   public static Supplier<YOLOv8ModelInfoPubSubType> getPubSubType()
   {
      return YOLOv8ModelInfoPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ModelInfoPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ModelInfo other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.model_name_, other.model_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.detectable_object_classes_, other.detectable_object_classes_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ModelInfo)) return false;

      YOLOv8ModelInfo otherMyClass = (YOLOv8ModelInfo) other;

      if (!us.ihmc.idl.IDLTools.equals(this.model_name_, otherMyClass.model_name_)) return false;

      if (!this.detectable_object_classes_.equals(otherMyClass.detectable_object_classes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ModelInfo {");
      builder.append("model_name=");
      builder.append(this.model_name_);      builder.append(", ");
      builder.append("detectable_object_classes=");
      builder.append(this.detectable_object_classes_);
      builder.append("}");
      return builder.toString();
   }
}
