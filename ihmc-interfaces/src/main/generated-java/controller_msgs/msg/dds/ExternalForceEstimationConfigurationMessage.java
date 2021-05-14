package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC external force estimation module.
       * It specifies where an expected external force will be applied.
       */
public class ExternalForceEstimationConfigurationMessage extends Packet<ExternalForceEstimationConfigurationMessage> implements Settable<ExternalForceEstimationConfigurationMessage>, EpsilonComparable<ExternalForceEstimationConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Estimator gain, usually in the range of 0.25 - 5.0. If the system is noisy or prone to oscillation, a lower gain is suggested.
            */
   public double estimator_gain_ = 0.5;
   /**
            * Specifies the alpha value used by the damped least-squares solver, usually in the range (1e-6 - 1e-2). For long joint paths, smaller alphas are recommended.
            */
   public double solver_alpha_ = 0.005;
   /**
            * Indicates whether the wrench at the root joint should be included in the solver
            */
   public boolean calculate_root_joint_wrench_ = true;
   /**
            * List of unique hash codes corresponding to the rigid bodies at which the solver will calculate external forces.
            * See RigidBody.hashCode() for calculation of the hash code
            */
   public us.ihmc.idl.IDLSequence.Integer  rigid_body_hash_codes_;
   /**
            * List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  contact_point_positions_;
   /**
            * (in beta) Estimates the location of the contact point at a single rigid body (the first provided). The root joint wrench is not estimated.
            */
   public boolean estimate_contact_location_;

   public ExternalForceEstimationConfigurationMessage()
   {
      rigid_body_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (10, "type_2");

      contact_point_positions_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public ExternalForceEstimationConfigurationMessage(ExternalForceEstimationConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(ExternalForceEstimationConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      estimator_gain_ = other.estimator_gain_;

      solver_alpha_ = other.solver_alpha_;

      calculate_root_joint_wrench_ = other.calculate_root_joint_wrench_;

      rigid_body_hash_codes_.set(other.rigid_body_hash_codes_);
      contact_point_positions_.set(other.contact_point_positions_);
      estimate_contact_location_ = other.estimate_contact_location_;

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
            * Estimator gain, usually in the range of 0.25 - 5.0. If the system is noisy or prone to oscillation, a lower gain is suggested.
            */
   public void setEstimatorGain(double estimator_gain)
   {
      estimator_gain_ = estimator_gain;
   }
   /**
            * Estimator gain, usually in the range of 0.25 - 5.0. If the system is noisy or prone to oscillation, a lower gain is suggested.
            */
   public double getEstimatorGain()
   {
      return estimator_gain_;
   }

   /**
            * Specifies the alpha value used by the damped least-squares solver, usually in the range (1e-6 - 1e-2). For long joint paths, smaller alphas are recommended.
            */
   public void setSolverAlpha(double solver_alpha)
   {
      solver_alpha_ = solver_alpha;
   }
   /**
            * Specifies the alpha value used by the damped least-squares solver, usually in the range (1e-6 - 1e-2). For long joint paths, smaller alphas are recommended.
            */
   public double getSolverAlpha()
   {
      return solver_alpha_;
   }

   /**
            * Indicates whether the wrench at the root joint should be included in the solver
            */
   public void setCalculateRootJointWrench(boolean calculate_root_joint_wrench)
   {
      calculate_root_joint_wrench_ = calculate_root_joint_wrench;
   }
   /**
            * Indicates whether the wrench at the root joint should be included in the solver
            */
   public boolean getCalculateRootJointWrench()
   {
      return calculate_root_joint_wrench_;
   }


   /**
            * List of unique hash codes corresponding to the rigid bodies at which the solver will calculate external forces.
            * See RigidBody.hashCode() for calculation of the hash code
            */
   public us.ihmc.idl.IDLSequence.Integer  getRigidBodyHashCodes()
   {
      return rigid_body_hash_codes_;
   }


   /**
            * List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getContactPointPositions()
   {
      return contact_point_positions_;
   }

   /**
            * (in beta) Estimates the location of the contact point at a single rigid body (the first provided). The root joint wrench is not estimated.
            */
   public void setEstimateContactLocation(boolean estimate_contact_location)
   {
      estimate_contact_location_ = estimate_contact_location;
   }
   /**
            * (in beta) Estimates the location of the contact point at a single rigid body (the first provided). The root joint wrench is not estimated.
            */
   public boolean getEstimateContactLocation()
   {
      return estimate_contact_location_;
   }


   public static Supplier<ExternalForceEstimationConfigurationMessagePubSubType> getPubSubType()
   {
      return ExternalForceEstimationConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExternalForceEstimationConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExternalForceEstimationConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.estimator_gain_, other.estimator_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solver_alpha_, other.solver_alpha_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.calculate_root_joint_wrench_, other.calculate_root_joint_wrench_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.rigid_body_hash_codes_, other.rigid_body_hash_codes_, epsilon)) return false;

      if (this.contact_point_positions_.size() != other.contact_point_positions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_point_positions_.size(); i++)
         {  if (!this.contact_point_positions_.get(i).epsilonEquals(other.contact_point_positions_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.estimate_contact_location_, other.estimate_contact_location_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExternalForceEstimationConfigurationMessage)) return false;

      ExternalForceEstimationConfigurationMessage otherMyClass = (ExternalForceEstimationConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.estimator_gain_ != otherMyClass.estimator_gain_) return false;

      if(this.solver_alpha_ != otherMyClass.solver_alpha_) return false;

      if(this.calculate_root_joint_wrench_ != otherMyClass.calculate_root_joint_wrench_) return false;

      if (!this.rigid_body_hash_codes_.equals(otherMyClass.rigid_body_hash_codes_)) return false;
      if (!this.contact_point_positions_.equals(otherMyClass.contact_point_positions_)) return false;
      if(this.estimate_contact_location_ != otherMyClass.estimate_contact_location_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExternalForceEstimationConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("estimator_gain=");
      builder.append(this.estimator_gain_);      builder.append(", ");
      builder.append("solver_alpha=");
      builder.append(this.solver_alpha_);      builder.append(", ");
      builder.append("calculate_root_joint_wrench=");
      builder.append(this.calculate_root_joint_wrench_);      builder.append(", ");
      builder.append("rigid_body_hash_codes=");
      builder.append(this.rigid_body_hash_codes_);      builder.append(", ");
      builder.append("contact_point_positions=");
      builder.append(this.contact_point_positions_);      builder.append(", ");
      builder.append("estimate_contact_location=");
      builder.append(this.estimate_contact_location_);
      builder.append("}");
      return builder.toString();
   }
}
