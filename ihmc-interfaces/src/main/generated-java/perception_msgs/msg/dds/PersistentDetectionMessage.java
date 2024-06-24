package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PersistentDetectionMessage extends Packet<PersistentDetectionMessage> implements Settable<PersistentDetectionMessage>, EpsilonComparable<PersistentDetectionMessage>
{
   /**
            * History
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.InstantDetectionMessage>  detection_history_;
   public ihmc_common_msgs.msg.dds.DurationMessage history_duration_;
   /**
            * First detections
            */
   public perception_msgs.msg.dds.InstantDetectionMessage first_detection_;
   /**
            * Stability thresholds
            */
   public double stability_confidence_threshold_;
   public double stability_detection_frequency_;
   public boolean ready_for_destruction_;

   public PersistentDetectionMessage()
   {
      detection_history_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.InstantDetectionMessage> (31, new perception_msgs.msg.dds.InstantDetectionMessagePubSubType());
      history_duration_ = new ihmc_common_msgs.msg.dds.DurationMessage();
      first_detection_ = new perception_msgs.msg.dds.InstantDetectionMessage();

   }

   public PersistentDetectionMessage(PersistentDetectionMessage other)
   {
      this();
      set(other);
   }

   public void set(PersistentDetectionMessage other)
   {
      detection_history_.set(other.detection_history_);
      ihmc_common_msgs.msg.dds.DurationMessagePubSubType.staticCopy(other.history_duration_, history_duration_);
      perception_msgs.msg.dds.InstantDetectionMessagePubSubType.staticCopy(other.first_detection_, first_detection_);
      stability_confidence_threshold_ = other.stability_confidence_threshold_;

      stability_detection_frequency_ = other.stability_detection_frequency_;

      ready_for_destruction_ = other.ready_for_destruction_;

   }


   /**
            * History
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.InstantDetectionMessage>  getDetectionHistory()
   {
      return detection_history_;
   }


   public ihmc_common_msgs.msg.dds.DurationMessage getHistoryDuration()
   {
      return history_duration_;
   }


   /**
            * First detections
            */
   public perception_msgs.msg.dds.InstantDetectionMessage getFirstDetection()
   {
      return first_detection_;
   }

   /**
            * Stability thresholds
            */
   public void setStabilityConfidenceThreshold(double stability_confidence_threshold)
   {
      stability_confidence_threshold_ = stability_confidence_threshold;
   }
   /**
            * Stability thresholds
            */
   public double getStabilityConfidenceThreshold()
   {
      return stability_confidence_threshold_;
   }

   public void setStabilityDetectionFrequency(double stability_detection_frequency)
   {
      stability_detection_frequency_ = stability_detection_frequency;
   }
   public double getStabilityDetectionFrequency()
   {
      return stability_detection_frequency_;
   }

   public void setReadyForDestruction(boolean ready_for_destruction)
   {
      ready_for_destruction_ = ready_for_destruction;
   }
   public boolean getReadyForDestruction()
   {
      return ready_for_destruction_;
   }


   public static Supplier<PersistentDetectionMessagePubSubType> getPubSubType()
   {
      return PersistentDetectionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PersistentDetectionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PersistentDetectionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.detection_history_.size() != other.detection_history_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detection_history_.size(); i++)
         {  if (!this.detection_history_.get(i).epsilonEquals(other.detection_history_.get(i), epsilon)) return false; }
      }

      if (!this.history_duration_.epsilonEquals(other.history_duration_, epsilon)) return false;
      if (!this.first_detection_.epsilonEquals(other.first_detection_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stability_confidence_threshold_, other.stability_confidence_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stability_detection_frequency_, other.stability_detection_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.ready_for_destruction_, other.ready_for_destruction_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PersistentDetectionMessage)) return false;

      PersistentDetectionMessage otherMyClass = (PersistentDetectionMessage) other;

      if (!this.detection_history_.equals(otherMyClass.detection_history_)) return false;
      if (!this.history_duration_.equals(otherMyClass.history_duration_)) return false;
      if (!this.first_detection_.equals(otherMyClass.first_detection_)) return false;
      if(this.stability_confidence_threshold_ != otherMyClass.stability_confidence_threshold_) return false;

      if(this.stability_detection_frequency_ != otherMyClass.stability_detection_frequency_) return false;

      if(this.ready_for_destruction_ != otherMyClass.ready_for_destruction_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PersistentDetectionMessage {");
      builder.append("detection_history=");
      builder.append(this.detection_history_);      builder.append(", ");
      builder.append("history_duration=");
      builder.append(this.history_duration_);      builder.append(", ");
      builder.append("first_detection=");
      builder.append(this.first_detection_);      builder.append(", ");
      builder.append("stability_confidence_threshold=");
      builder.append(this.stability_confidence_threshold_);      builder.append(", ");
      builder.append("stability_detection_frequency=");
      builder.append(this.stability_detection_frequency_);      builder.append(", ");
      builder.append("ready_for_destruction=");
      builder.append(this.ready_for_destruction_);
      builder.append("}");
      return builder.toString();
   }
}
