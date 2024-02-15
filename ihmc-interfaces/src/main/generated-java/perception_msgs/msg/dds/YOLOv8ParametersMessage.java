package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Parameters for running YOLOv8
       */
public class YOLOv8ParametersMessage extends Packet<YOLOv8ParametersMessage> implements Settable<YOLOv8ParametersMessage>, EpsilonComparable<YOLOv8ParametersMessage>
{
   /**
            * Confidence required to consider a mask a valid detection
            */
   public float confidence_threshold_;
   public float non_maximum_suppression_threshold_;
   public float segmentation_threshold_;
   /**
            * Squared Euclidean distance (in pixels) a candidate detection can move
            */
   public float candidate_movement_threshold_;
   /**
            * Percentage of updates a candidate must be detected in to be accepted
            */
   public float candidate_acceptance_threshold_;

   public YOLOv8ParametersMessage()
   {
   }

   public YOLOv8ParametersMessage(YOLOv8ParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ParametersMessage other)
   {
      confidence_threshold_ = other.confidence_threshold_;

      non_maximum_suppression_threshold_ = other.non_maximum_suppression_threshold_;

      segmentation_threshold_ = other.segmentation_threshold_;

      candidate_movement_threshold_ = other.candidate_movement_threshold_;

      candidate_acceptance_threshold_ = other.candidate_acceptance_threshold_;

   }

   /**
            * Confidence required to consider a mask a valid detection
            */
   public void setConfidenceThreshold(float confidence_threshold)
   {
      confidence_threshold_ = confidence_threshold;
   }
   /**
            * Confidence required to consider a mask a valid detection
            */
   public float getConfidenceThreshold()
   {
      return confidence_threshold_;
   }

   public void setNonMaximumSuppressionThreshold(float non_maximum_suppression_threshold)
   {
      non_maximum_suppression_threshold_ = non_maximum_suppression_threshold;
   }
   public float getNonMaximumSuppressionThreshold()
   {
      return non_maximum_suppression_threshold_;
   }

   public void setSegmentationThreshold(float segmentation_threshold)
   {
      segmentation_threshold_ = segmentation_threshold;
   }
   public float getSegmentationThreshold()
   {
      return segmentation_threshold_;
   }

   /**
            * Squared Euclidean distance (in pixels) a candidate detection can move
            */
   public void setCandidateMovementThreshold(float candidate_movement_threshold)
   {
      candidate_movement_threshold_ = candidate_movement_threshold;
   }
   /**
            * Squared Euclidean distance (in pixels) a candidate detection can move
            */
   public float getCandidateMovementThreshold()
   {
      return candidate_movement_threshold_;
   }

   /**
            * Percentage of updates a candidate must be detected in to be accepted
            */
   public void setCandidateAcceptanceThreshold(float candidate_acceptance_threshold)
   {
      candidate_acceptance_threshold_ = candidate_acceptance_threshold;
   }
   /**
            * Percentage of updates a candidate must be detected in to be accepted
            */
   public float getCandidateAcceptanceThreshold()
   {
      return candidate_acceptance_threshold_;
   }


   public static Supplier<YOLOv8ParametersMessagePubSubType> getPubSubType()
   {
      return YOLOv8ParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_threshold_, other.confidence_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.non_maximum_suppression_threshold_, other.non_maximum_suppression_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segmentation_threshold_, other.segmentation_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.candidate_movement_threshold_, other.candidate_movement_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.candidate_acceptance_threshold_, other.candidate_acceptance_threshold_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ParametersMessage)) return false;

      YOLOv8ParametersMessage otherMyClass = (YOLOv8ParametersMessage) other;

      if(this.confidence_threshold_ != otherMyClass.confidence_threshold_) return false;

      if(this.non_maximum_suppression_threshold_ != otherMyClass.non_maximum_suppression_threshold_) return false;

      if(this.segmentation_threshold_ != otherMyClass.segmentation_threshold_) return false;

      if(this.candidate_movement_threshold_ != otherMyClass.candidate_movement_threshold_) return false;

      if(this.candidate_acceptance_threshold_ != otherMyClass.candidate_acceptance_threshold_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ParametersMessage {");
      builder.append("confidence_threshold=");
      builder.append(this.confidence_threshold_);      builder.append(", ");
      builder.append("non_maximum_suppression_threshold=");
      builder.append(this.non_maximum_suppression_threshold_);      builder.append(", ");
      builder.append("segmentation_threshold=");
      builder.append(this.segmentation_threshold_);      builder.append(", ");
      builder.append("candidate_movement_threshold=");
      builder.append(this.candidate_movement_threshold_);      builder.append(", ");
      builder.append("candidate_acceptance_threshold=");
      builder.append(this.candidate_acceptance_threshold_);
      builder.append("}");
      return builder.toString();
   }
}
