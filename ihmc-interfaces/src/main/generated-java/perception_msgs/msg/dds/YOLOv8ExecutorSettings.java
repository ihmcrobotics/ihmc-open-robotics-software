package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Settings for the YOLOv8DetectionExecutor class
       */
public class YOLOv8ExecutorSettings extends Packet<YOLOv8ExecutorSettings> implements Settable<YOLOv8ExecutorSettings>, EpsilonComparable<YOLOv8ExecutorSettings>
{
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   /**
            * List of models to run
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  models_to_run_;
   /**
            * Settings for each model
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelSettings>  model_settings_;

   public YOLOv8ExecutorSettings()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      models_to_run_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (16, "type_d");
      model_settings_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelSettings> (16, new perception_msgs.msg.dds.YOLOv8ModelSettingsPubSubType());

   }

   public YOLOv8ExecutorSettings(YOLOv8ExecutorSettings other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ExecutorSettings other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      models_to_run_.set(other.models_to_run_);
      model_settings_.set(other.model_settings_);
   }


   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }


   /**
            * List of models to run
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getModelsToRun()
   {
      return models_to_run_;
   }


   /**
            * Settings for each model
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelSettings>  getModelSettings()
   {
      return model_settings_;
   }


   public static Supplier<YOLOv8ExecutorSettingsPubSubType> getPubSubType()
   {
      return YOLOv8ExecutorSettingsPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ExecutorSettingsPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ExecutorSettings other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.models_to_run_, other.models_to_run_, epsilon)) return false;

      if (this.model_settings_.size() != other.model_settings_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.model_settings_.size(); i++)
         {  if (!this.model_settings_.get(i).epsilonEquals(other.model_settings_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ExecutorSettings)) return false;

      YOLOv8ExecutorSettings otherMyClass = (YOLOv8ExecutorSettings) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if (!this.models_to_run_.equals(otherMyClass.models_to_run_)) return false;
      if (!this.model_settings_.equals(otherMyClass.model_settings_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ExecutorSettings {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("models_to_run=");
      builder.append(this.models_to_run_);      builder.append(", ");
      builder.append("model_settings=");
      builder.append(this.model_settings_);
      builder.append("}");
      return builder.toString();
   }
}
