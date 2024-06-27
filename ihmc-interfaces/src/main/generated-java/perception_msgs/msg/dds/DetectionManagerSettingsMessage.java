package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * TODO: Having these parameter/settings messages kinda sucks.
       * Should try to get a better system.
       */
public class DetectionManagerSettingsMessage extends Packet<DetectionManagerSettingsMessage> implements Settable<DetectionManagerSettingsMessage>, EpsilonComparable<DetectionManagerSettingsMessage>
{
   public float match_distance_threshold_;
   public float acceptance_average_confidence_;
   public float stability_average_confidence_;
   public float stability_frequency_;
   public float detection_history_duration_;

   public DetectionManagerSettingsMessage()
   {
   }

   public DetectionManagerSettingsMessage(DetectionManagerSettingsMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectionManagerSettingsMessage other)
   {
      match_distance_threshold_ = other.match_distance_threshold_;

      acceptance_average_confidence_ = other.acceptance_average_confidence_;

      stability_average_confidence_ = other.stability_average_confidence_;

      stability_frequency_ = other.stability_frequency_;

      detection_history_duration_ = other.detection_history_duration_;

   }

   public void setMatchDistanceThreshold(float match_distance_threshold)
   {
      match_distance_threshold_ = match_distance_threshold;
   }
   public float getMatchDistanceThreshold()
   {
      return match_distance_threshold_;
   }

   public void setAcceptanceAverageConfidence(float acceptance_average_confidence)
   {
      acceptance_average_confidence_ = acceptance_average_confidence;
   }
   public float getAcceptanceAverageConfidence()
   {
      return acceptance_average_confidence_;
   }

   public void setStabilityAverageConfidence(float stability_average_confidence)
   {
      stability_average_confidence_ = stability_average_confidence;
   }
   public float getStabilityAverageConfidence()
   {
      return stability_average_confidence_;
   }

   public void setStabilityFrequency(float stability_frequency)
   {
      stability_frequency_ = stability_frequency;
   }
   public float getStabilityFrequency()
   {
      return stability_frequency_;
   }

   public void setDetectionHistoryDuration(float detection_history_duration)
   {
      detection_history_duration_ = detection_history_duration;
   }
   public float getDetectionHistoryDuration()
   {
      return detection_history_duration_;
   }


   public static Supplier<DetectionManagerSettingsMessagePubSubType> getPubSubType()
   {
      return DetectionManagerSettingsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectionManagerSettingsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectionManagerSettingsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.match_distance_threshold_, other.match_distance_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acceptance_average_confidence_, other.acceptance_average_confidence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stability_average_confidence_, other.stability_average_confidence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stability_frequency_, other.stability_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.detection_history_duration_, other.detection_history_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectionManagerSettingsMessage)) return false;

      DetectionManagerSettingsMessage otherMyClass = (DetectionManagerSettingsMessage) other;

      if(this.match_distance_threshold_ != otherMyClass.match_distance_threshold_) return false;

      if(this.acceptance_average_confidence_ != otherMyClass.acceptance_average_confidence_) return false;

      if(this.stability_average_confidence_ != otherMyClass.stability_average_confidence_) return false;

      if(this.stability_frequency_ != otherMyClass.stability_frequency_) return false;

      if(this.detection_history_duration_ != otherMyClass.detection_history_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectionManagerSettingsMessage {");
      builder.append("match_distance_threshold=");
      builder.append(this.match_distance_threshold_);      builder.append(", ");
      builder.append("acceptance_average_confidence=");
      builder.append(this.acceptance_average_confidence_);      builder.append(", ");
      builder.append("stability_average_confidence=");
      builder.append(this.stability_average_confidence_);      builder.append(", ");
      builder.append("stability_frequency=");
      builder.append(this.stability_frequency_);      builder.append(", ");
      builder.append("detection_history_duration=");
      builder.append(this.detection_history_duration_);
      builder.append("}");
      return builder.toString();
   }
}
