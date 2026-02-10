package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Settings for the YOLOv8DetectionExecutor class
       */
public class YOLOv8ExecutorParameters extends Packet<YOLOv8ExecutorParameters> implements Settable<YOLOv8ExecutorParameters>, EpsilonComparable<YOLOv8ExecutorParameters>
{
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   /**
            * List of the available models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo>  available_yolo_models_;
   /**
            * List of models to run
            */
   public java.lang.StringBuilder model_to_run_;
   /**
            * Settings for each model
            */
   public perception_msgs.msg.dds.YOLOv8ModelParameters model_settings_;

   public YOLOv8ExecutorParameters()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      available_yolo_models_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo> (16, new perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType());
      model_to_run_ = new java.lang.StringBuilder(255);
      model_settings_ = new perception_msgs.msg.dds.YOLOv8ModelParameters();

   }

   public YOLOv8ExecutorParameters(YOLOv8ExecutorParameters other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ExecutorParameters other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      available_yolo_models_.set(other.available_yolo_models_);
      model_to_run_.setLength(0);
      model_to_run_.append(other.model_to_run_);

      perception_msgs.msg.dds.YOLOv8ModelParametersPubSubType.staticCopy(other.model_settings_, model_settings_);
   }


   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }


   /**
            * List of the available models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelInfo>  getAvailableYoloModels()
   {
      return available_yolo_models_;
   }

   /**
            * List of models to run
            */
   public void setModelToRun(java.lang.String model_to_run)
   {
      model_to_run_.setLength(0);
      model_to_run_.append(model_to_run);
   }

   /**
            * List of models to run
            */
   public java.lang.String getModelToRunAsString()
   {
      return getModelToRun().toString();
   }
   /**
            * List of models to run
            */
   public java.lang.StringBuilder getModelToRun()
   {
      return model_to_run_;
   }


   /**
            * Settings for each model
            */
   public perception_msgs.msg.dds.YOLOv8ModelParameters getModelSettings()
   {
      return model_settings_;
   }


   public static Supplier<YOLOv8ExecutorParametersPubSubType> getPubSubType()
   {
      return YOLOv8ExecutorParametersPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ExecutorParametersPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ExecutorParameters other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (this.available_yolo_models_.size() != other.available_yolo_models_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.available_yolo_models_.size(); i++)
         {  if (!this.available_yolo_models_.get(i).epsilonEquals(other.available_yolo_models_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.model_to_run_, other.model_to_run_, epsilon)) return false;

      if (!this.model_settings_.epsilonEquals(other.model_settings_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ExecutorParameters)) return false;

      YOLOv8ExecutorParameters otherMyClass = (YOLOv8ExecutorParameters) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if (!this.available_yolo_models_.equals(otherMyClass.available_yolo_models_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.model_to_run_, otherMyClass.model_to_run_)) return false;

      if (!this.model_settings_.equals(otherMyClass.model_settings_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ExecutorParameters {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("available_yolo_models=");
      builder.append(this.available_yolo_models_);      builder.append(", ");
      builder.append("model_to_run=");
      builder.append(this.model_to_run_);      builder.append(", ");
      builder.append("model_settings=");
      builder.append(this.model_settings_);
      builder.append("}");
      return builder.toString();
   }
}
