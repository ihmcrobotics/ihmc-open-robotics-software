package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC external force estimation module.
       * It provides the result from the estimator
       */
public class ExternalForceEstimationOutputStatus extends Packet<ExternalForceEstimationOutputStatus> implements Settable<ExternalForceEstimationOutputStatus>, EpsilonComparable<ExternalForceEstimationOutputStatus>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Estimated external force in world frame
            */
   public us.ihmc.euclid.tuple3D.Vector3D estimated_external_force_;
   /**
            * An indicator of the quality of the estimation
            */
   public double solution_quality_ = -1.0;

   public ExternalForceEstimationOutputStatus()
   {
      estimated_external_force_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public ExternalForceEstimationOutputStatus(ExternalForceEstimationOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(ExternalForceEstimationOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.estimated_external_force_, estimated_external_force_);
      solution_quality_ = other.solution_quality_;

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


   /**
            * Estimated external force in world frame
            */
   public us.ihmc.euclid.tuple3D.Vector3D getEstimatedExternalForce()
   {
      return estimated_external_force_;
   }

   /**
            * An indicator of the quality of the estimation
            */
   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   /**
            * An indicator of the quality of the estimation
            */
   public double getSolutionQuality()
   {
      return solution_quality_;
   }


   public static Supplier<ExternalForceEstimationOutputStatusPubSubType> getPubSubType()
   {
      return ExternalForceEstimationOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExternalForceEstimationOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExternalForceEstimationOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.estimated_external_force_.epsilonEquals(other.estimated_external_force_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExternalForceEstimationOutputStatus)) return false;

      ExternalForceEstimationOutputStatus otherMyClass = (ExternalForceEstimationOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.estimated_external_force_.equals(otherMyClass.estimated_external_force_)) return false;
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExternalForceEstimationOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("estimated_external_force=");
      builder.append(this.estimated_external_force_);      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);
      builder.append("}");
      return builder.toString();
   }
}
