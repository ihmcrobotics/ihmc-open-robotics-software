package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message listing available YOLO models
       */
public class YOLOv8AvailableModels extends Packet<YOLOv8AvailableModels> implements Settable<YOLOv8AvailableModels>, EpsilonComparable<YOLOv8AvailableModels>
{
   /**
            * Whether this is a request to receive the available models
            */
   public boolean request_;
   /**
            * List of the available models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo>  available_yolo_models_;

   public YOLOv8AvailableModels()
   {
      available_yolo_models_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo> (8, new perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType());

   }

   public YOLOv8AvailableModels(YOLOv8AvailableModels other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8AvailableModels other)
   {
      request_ = other.request_;

      available_yolo_models_.set(other.available_yolo_models_);
   }

   /**
            * Whether this is a request to receive the available models
            */
   public void setRequest(boolean request)
   {
      request_ = request;
   }
   /**
            * Whether this is a request to receive the available models
            */
   public boolean getRequest()
   {
      return request_;
   }


   /**
            * List of the available models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo>  getAvailableYoloModels()
   {
      return available_yolo_models_;
   }


   public static Supplier<YOLOv8AvailableModelsPubSubType> getPubSubType()
   {
      return YOLOv8AvailableModelsPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8AvailableModelsPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8AvailableModels other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_, other.request_, epsilon)) return false;

      if (this.available_yolo_models_.size() != other.available_yolo_models_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.available_yolo_models_.size(); i++)
         {  if (!this.available_yolo_models_.get(i).epsilonEquals(other.available_yolo_models_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8AvailableModels)) return false;

      YOLOv8AvailableModels otherMyClass = (YOLOv8AvailableModels) other;

      if(this.request_ != otherMyClass.request_) return false;

      if (!this.available_yolo_models_.equals(otherMyClass.available_yolo_models_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8AvailableModels {");
      builder.append("request=");
      builder.append(this.request_);      builder.append(", ");
      builder.append("available_yolo_models=");
      builder.append(this.available_yolo_models_);
      builder.append("}");
      return builder.toString();
   }
}
