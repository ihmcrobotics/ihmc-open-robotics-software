package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC Robot Environment Awareness (REA) module.
       */
public class NormalEstimationParametersMessage extends Packet<NormalEstimationParametersMessage> implements Settable<NormalEstimationParametersMessage>, EpsilonComparable<NormalEstimationParametersMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double search_radius_ = 0.08;
   public double max_distance_from_plane_ = 0.02;
   public double min_consensus_ratio_ = 0.5;
   public double max_average_deviation_ratio_ = 0.75;
   public int number_of_iterations_ = 1;
   public boolean enable_least_squares_estimation_ = true;
   public boolean weight_by_number_of_hits_ = true;

   public NormalEstimationParametersMessage()
   {
   }

   public NormalEstimationParametersMessage(NormalEstimationParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(NormalEstimationParametersMessage other)
   {
      sequence_id_ = other.sequence_id_;

      search_radius_ = other.search_radius_;

      max_distance_from_plane_ = other.max_distance_from_plane_;

      min_consensus_ratio_ = other.min_consensus_ratio_;

      max_average_deviation_ratio_ = other.max_average_deviation_ratio_;

      number_of_iterations_ = other.number_of_iterations_;

      enable_least_squares_estimation_ = other.enable_least_squares_estimation_;

      weight_by_number_of_hits_ = other.weight_by_number_of_hits_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setSearchRadius(double search_radius)
   {
      search_radius_ = search_radius;
   }
   public double getSearchRadius()
   {
      return search_radius_;
   }

   public void setMaxDistanceFromPlane(double max_distance_from_plane)
   {
      max_distance_from_plane_ = max_distance_from_plane;
   }
   public double getMaxDistanceFromPlane()
   {
      return max_distance_from_plane_;
   }

   public void setMinConsensusRatio(double min_consensus_ratio)
   {
      min_consensus_ratio_ = min_consensus_ratio;
   }
   public double getMinConsensusRatio()
   {
      return min_consensus_ratio_;
   }

   public void setMaxAverageDeviationRatio(double max_average_deviation_ratio)
   {
      max_average_deviation_ratio_ = max_average_deviation_ratio;
   }
   public double getMaxAverageDeviationRatio()
   {
      return max_average_deviation_ratio_;
   }

   public void setNumberOfIterations(int number_of_iterations)
   {
      number_of_iterations_ = number_of_iterations;
   }
   public int getNumberOfIterations()
   {
      return number_of_iterations_;
   }

   public void setEnableLeastSquaresEstimation(boolean enable_least_squares_estimation)
   {
      enable_least_squares_estimation_ = enable_least_squares_estimation;
   }
   public boolean getEnableLeastSquaresEstimation()
   {
      return enable_least_squares_estimation_;
   }

   public void setWeightByNumberOfHits(boolean weight_by_number_of_hits)
   {
      weight_by_number_of_hits_ = weight_by_number_of_hits;
   }
   public boolean getWeightByNumberOfHits()
   {
      return weight_by_number_of_hits_;
   }


   public static Supplier<NormalEstimationParametersMessagePubSubType> getPubSubType()
   {
      return NormalEstimationParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NormalEstimationParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NormalEstimationParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.search_radius_, other.search_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_distance_from_plane_, other.max_distance_from_plane_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_consensus_ratio_, other.min_consensus_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_average_deviation_ratio_, other.max_average_deviation_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_iterations_, other.number_of_iterations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_least_squares_estimation_, other.enable_least_squares_estimation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.weight_by_number_of_hits_, other.weight_by_number_of_hits_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NormalEstimationParametersMessage)) return false;

      NormalEstimationParametersMessage otherMyClass = (NormalEstimationParametersMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.search_radius_ != otherMyClass.search_radius_) return false;

      if(this.max_distance_from_plane_ != otherMyClass.max_distance_from_plane_) return false;

      if(this.min_consensus_ratio_ != otherMyClass.min_consensus_ratio_) return false;

      if(this.max_average_deviation_ratio_ != otherMyClass.max_average_deviation_ratio_) return false;

      if(this.number_of_iterations_ != otherMyClass.number_of_iterations_) return false;

      if(this.enable_least_squares_estimation_ != otherMyClass.enable_least_squares_estimation_) return false;

      if(this.weight_by_number_of_hits_ != otherMyClass.weight_by_number_of_hits_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NormalEstimationParametersMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("search_radius=");
      builder.append(this.search_radius_);      builder.append(", ");
      builder.append("max_distance_from_plane=");
      builder.append(this.max_distance_from_plane_);      builder.append(", ");
      builder.append("min_consensus_ratio=");
      builder.append(this.min_consensus_ratio_);      builder.append(", ");
      builder.append("max_average_deviation_ratio=");
      builder.append(this.max_average_deviation_ratio_);      builder.append(", ");
      builder.append("number_of_iterations=");
      builder.append(this.number_of_iterations_);      builder.append(", ");
      builder.append("enable_least_squares_estimation=");
      builder.append(this.enable_least_squares_estimation_);      builder.append(", ");
      builder.append("weight_by_number_of_hits=");
      builder.append(this.weight_by_number_of_hits_);
      builder.append("}");
      return builder.toString();
   }
}
