package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The state of the behavior tree scene
       */
public class BehaviorTreeSceneStateMessage extends Packet<BehaviorTreeSceneStateMessage> implements Settable<BehaviorTreeSceneStateMessage>, EpsilonComparable<BehaviorTreeSceneStateMessage>
{
   /**
            * The timestamp and modifier ID of the latest modification to the list of objects
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_list_;
   /**
            * Scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage>  objects_;
   /**
            * Persistent detections tracked in the scene
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PersistentDetectionStatusMessage>  persistent_detections_;
   public float pose_filter_alpha_;
   public float acceptance_confidence_;
   public float stability_frequency_;
   public float history_duration_;

   public BehaviorTreeSceneStateMessage()
   {
      latest_modification_to_list_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      objects_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage> (100, new behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessagePubSubType());
      persistent_detections_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PersistentDetectionStatusMessage> (100, new behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType());

   }

   public BehaviorTreeSceneStateMessage(BehaviorTreeSceneStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeSceneStateMessage other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_list_, latest_modification_to_list_);
      objects_.set(other.objects_);
      persistent_detections_.set(other.persistent_detections_);
      pose_filter_alpha_ = other.pose_filter_alpha_;

      acceptance_confidence_ = other.acceptance_confidence_;

      stability_frequency_ = other.stability_frequency_;

      history_duration_ = other.history_duration_;

   }


   /**
            * The timestamp and modifier ID of the latest modification to the list of objects
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToList()
   {
      return latest_modification_to_list_;
   }


   /**
            * Scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage>  getObjects()
   {
      return objects_;
   }


   /**
            * Persistent detections tracked in the scene
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PersistentDetectionStatusMessage>  getPersistentDetections()
   {
      return persistent_detections_;
   }

   public void setPoseFilterAlpha(float pose_filter_alpha)
   {
      pose_filter_alpha_ = pose_filter_alpha;
   }
   public float getPoseFilterAlpha()
   {
      return pose_filter_alpha_;
   }

   public void setAcceptanceConfidence(float acceptance_confidence)
   {
      acceptance_confidence_ = acceptance_confidence;
   }
   public float getAcceptanceConfidence()
   {
      return acceptance_confidence_;
   }

   public void setStabilityFrequency(float stability_frequency)
   {
      stability_frequency_ = stability_frequency;
   }
   public float getStabilityFrequency()
   {
      return stability_frequency_;
   }

   public void setHistoryDuration(float history_duration)
   {
      history_duration_ = history_duration;
   }
   public float getHistoryDuration()
   {
      return history_duration_;
   }


   public static Supplier<BehaviorTreeSceneStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeSceneStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeSceneStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeSceneStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_modification_to_list_.epsilonEquals(other.latest_modification_to_list_, epsilon)) return false;
      if (this.objects_.size() != other.objects_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.objects_.size(); i++)
         {  if (!this.objects_.get(i).epsilonEquals(other.objects_.get(i), epsilon)) return false; }
      }

      if (this.persistent_detections_.size() != other.persistent_detections_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.persistent_detections_.size(); i++)
         {  if (!this.persistent_detections_.get(i).epsilonEquals(other.persistent_detections_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pose_filter_alpha_, other.pose_filter_alpha_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acceptance_confidence_, other.acceptance_confidence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stability_frequency_, other.stability_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.history_duration_, other.history_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeSceneStateMessage)) return false;

      BehaviorTreeSceneStateMessage otherMyClass = (BehaviorTreeSceneStateMessage) other;

      if (!this.latest_modification_to_list_.equals(otherMyClass.latest_modification_to_list_)) return false;
      if (!this.objects_.equals(otherMyClass.objects_)) return false;
      if (!this.persistent_detections_.equals(otherMyClass.persistent_detections_)) return false;
      if(this.pose_filter_alpha_ != otherMyClass.pose_filter_alpha_) return false;

      if(this.acceptance_confidence_ != otherMyClass.acceptance_confidence_) return false;

      if(this.stability_frequency_ != otherMyClass.stability_frequency_) return false;

      if(this.history_duration_ != otherMyClass.history_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeSceneStateMessage {");
      builder.append("latest_modification_to_list=");
      builder.append(this.latest_modification_to_list_);      builder.append(", ");
      builder.append("objects=");
      builder.append(this.objects_);      builder.append(", ");
      builder.append("persistent_detections=");
      builder.append(this.persistent_detections_);      builder.append(", ");
      builder.append("pose_filter_alpha=");
      builder.append(this.pose_filter_alpha_);      builder.append(", ");
      builder.append("acceptance_confidence=");
      builder.append(this.acceptance_confidence_);      builder.append(", ");
      builder.append("stability_frequency=");
      builder.append(this.stability_frequency_);      builder.append(", ");
      builder.append("history_duration=");
      builder.append(this.history_duration_);
      builder.append("}");
      return builder.toString();
   }
}
