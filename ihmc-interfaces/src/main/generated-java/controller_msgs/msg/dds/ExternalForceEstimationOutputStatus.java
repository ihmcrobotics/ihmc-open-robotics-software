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
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  estimated_external_forces_;
   /**
            * Estimated root joint wrench, if requested. Will be set to NaN if root joint was not included in the solver
            */
   public geometry_msgs.msg.dds.Wrench estimated_root_joint_wrench_;
   /**
            * Estimated contacting rigid body hashcode, if configured with estimate_contact_location=true
            */
   public int rigid_body_hash_code_;
   /**
            * Estimated contact point in parent joint frame, if configured with estimate_contact_location=true
            */
   public us.ihmc.euclid.tuple3D.Point3D contact_point_;

   public ExternalForceEstimationOutputStatus()
   {
      estimated_external_forces_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (10, new geometry_msgs.msg.dds.Vector3PubSubType());
      estimated_root_joint_wrench_ = new geometry_msgs.msg.dds.Wrench();
      contact_point_ = new us.ihmc.euclid.tuple3D.Point3D();

   }

   public ExternalForceEstimationOutputStatus(ExternalForceEstimationOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(ExternalForceEstimationOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      estimated_external_forces_.set(other.estimated_external_forces_);
      geometry_msgs.msg.dds.WrenchPubSubType.staticCopy(other.estimated_root_joint_wrench_, estimated_root_joint_wrench_);
      rigid_body_hash_code_ = other.rigid_body_hash_code_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.contact_point_, contact_point_);
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
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getEstimatedExternalForces()
   {
      return estimated_external_forces_;
   }


   /**
            * Estimated root joint wrench, if requested. Will be set to NaN if root joint was not included in the solver
            */
   public geometry_msgs.msg.dds.Wrench getEstimatedRootJointWrench()
   {
      return estimated_root_joint_wrench_;
   }

   /**
            * Estimated contacting rigid body hashcode, if configured with estimate_contact_location=true
            */
   public void setRigidBodyHashCode(int rigid_body_hash_code)
   {
      rigid_body_hash_code_ = rigid_body_hash_code;
   }
   /**
            * Estimated contacting rigid body hashcode, if configured with estimate_contact_location=true
            */
   public int getRigidBodyHashCode()
   {
      return rigid_body_hash_code_;
   }


   /**
            * Estimated contact point in parent joint frame, if configured with estimate_contact_location=true
            */
   public us.ihmc.euclid.tuple3D.Point3D getContactPoint()
   {
      return contact_point_;
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

      if (this.estimated_external_forces_.size() != other.estimated_external_forces_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.estimated_external_forces_.size(); i++)
         {  if (!this.estimated_external_forces_.get(i).epsilonEquals(other.estimated_external_forces_.get(i), epsilon)) return false; }
      }

      if (!this.estimated_root_joint_wrench_.epsilonEquals(other.estimated_root_joint_wrench_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rigid_body_hash_code_, other.rigid_body_hash_code_, epsilon)) return false;

      if (!this.contact_point_.epsilonEquals(other.contact_point_, epsilon)) return false;

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

      if (!this.estimated_external_forces_.equals(otherMyClass.estimated_external_forces_)) return false;
      if (!this.estimated_root_joint_wrench_.equals(otherMyClass.estimated_root_joint_wrench_)) return false;
      if(this.rigid_body_hash_code_ != otherMyClass.rigid_body_hash_code_) return false;

      if (!this.contact_point_.equals(otherMyClass.contact_point_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExternalForceEstimationOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("estimated_external_forces=");
      builder.append(this.estimated_external_forces_);      builder.append(", ");
      builder.append("estimated_root_joint_wrench=");
      builder.append(this.estimated_root_joint_wrench_);      builder.append(", ");
      builder.append("rigid_body_hash_code=");
      builder.append(this.rigid_body_hash_code_);      builder.append(", ");
      builder.append("contact_point=");
      builder.append(this.contact_point_);
      builder.append("}");
      return builder.toString();
   }
}
