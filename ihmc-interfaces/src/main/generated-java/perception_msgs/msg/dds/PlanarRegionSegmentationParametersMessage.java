package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC Robot Environment Awareness (REA) module.
       */
public class PlanarRegionSegmentationParametersMessage extends Packet<PlanarRegionSegmentationParametersMessage> implements Settable<PlanarRegionSegmentationParametersMessage>, EpsilonComparable<PlanarRegionSegmentationParametersMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double search_radius_ = 0.05;
   public double max_distance_from_plane_ = 0.05;
   public double max_angle_from_plane_ = 0.1745;
   public double min_normal_quality_ = 0.005;
   public int min_region_size_ = 50;
   public double max_standard_deviation_ = 0.015;
   /**
            * In units of voxels per m^3. Default value is equivalent to 0.1 / cm^3
            */
   public double min_volumic_density_ = 100000.0;

   public PlanarRegionSegmentationParametersMessage()
   {
   }

   public PlanarRegionSegmentationParametersMessage(PlanarRegionSegmentationParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(PlanarRegionSegmentationParametersMessage other)
   {
      sequence_id_ = other.sequence_id_;

      search_radius_ = other.search_radius_;

      max_distance_from_plane_ = other.max_distance_from_plane_;

      max_angle_from_plane_ = other.max_angle_from_plane_;

      min_normal_quality_ = other.min_normal_quality_;

      min_region_size_ = other.min_region_size_;

      max_standard_deviation_ = other.max_standard_deviation_;

      min_volumic_density_ = other.min_volumic_density_;

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

   public void setMaxAngleFromPlane(double max_angle_from_plane)
   {
      max_angle_from_plane_ = max_angle_from_plane;
   }
   public double getMaxAngleFromPlane()
   {
      return max_angle_from_plane_;
   }

   public void setMinNormalQuality(double min_normal_quality)
   {
      min_normal_quality_ = min_normal_quality;
   }
   public double getMinNormalQuality()
   {
      return min_normal_quality_;
   }

   public void setMinRegionSize(int min_region_size)
   {
      min_region_size_ = min_region_size;
   }
   public int getMinRegionSize()
   {
      return min_region_size_;
   }

   public void setMaxStandardDeviation(double max_standard_deviation)
   {
      max_standard_deviation_ = max_standard_deviation;
   }
   public double getMaxStandardDeviation()
   {
      return max_standard_deviation_;
   }

   /**
            * In units of voxels per m^3. Default value is equivalent to 0.1 / cm^3
            */
   public void setMinVolumicDensity(double min_volumic_density)
   {
      min_volumic_density_ = min_volumic_density;
   }
   /**
            * In units of voxels per m^3. Default value is equivalent to 0.1 / cm^3
            */
   public double getMinVolumicDensity()
   {
      return min_volumic_density_;
   }


   public static Supplier<PlanarRegionSegmentationParametersMessagePubSubType> getPubSubType()
   {
      return PlanarRegionSegmentationParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PlanarRegionSegmentationParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionSegmentationParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.search_radius_, other.search_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_distance_from_plane_, other.max_distance_from_plane_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_angle_from_plane_, other.max_angle_from_plane_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_normal_quality_, other.min_normal_quality_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_region_size_, other.min_region_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_standard_deviation_, other.max_standard_deviation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_volumic_density_, other.min_volumic_density_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PlanarRegionSegmentationParametersMessage)) return false;

      PlanarRegionSegmentationParametersMessage otherMyClass = (PlanarRegionSegmentationParametersMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.search_radius_ != otherMyClass.search_radius_) return false;

      if(this.max_distance_from_plane_ != otherMyClass.max_distance_from_plane_) return false;

      if(this.max_angle_from_plane_ != otherMyClass.max_angle_from_plane_) return false;

      if(this.min_normal_quality_ != otherMyClass.min_normal_quality_) return false;

      if(this.min_region_size_ != otherMyClass.min_region_size_) return false;

      if(this.max_standard_deviation_ != otherMyClass.max_standard_deviation_) return false;

      if(this.min_volumic_density_ != otherMyClass.min_volumic_density_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanarRegionSegmentationParametersMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("search_radius=");
      builder.append(this.search_radius_);      builder.append(", ");
      builder.append("max_distance_from_plane=");
      builder.append(this.max_distance_from_plane_);      builder.append(", ");
      builder.append("max_angle_from_plane=");
      builder.append(this.max_angle_from_plane_);      builder.append(", ");
      builder.append("min_normal_quality=");
      builder.append(this.min_normal_quality_);      builder.append(", ");
      builder.append("min_region_size=");
      builder.append(this.min_region_size_);      builder.append(", ");
      builder.append("max_standard_deviation=");
      builder.append(this.max_standard_deviation_);      builder.append(", ");
      builder.append("min_volumic_density=");
      builder.append(this.min_volumic_density_);
      builder.append("}");
      return builder.toString();
   }
}
