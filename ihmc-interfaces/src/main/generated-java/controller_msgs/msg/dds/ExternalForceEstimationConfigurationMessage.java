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
   public double estimator_gain_;
   /**
            * The is the unique hash code of the end-effector that is assumed to be pushed
            * See RigidBody.hashCode() for the computation of a rigid-body hash code.
            */
   public int end_effector_hash_code_;
   /**
            * This is the position externally applied force to be estimated, in endEffector.getBodyFixedFrame().
            */
   public us.ihmc.euclid.tuple3D.Point3D external_force_position_;

   public ExternalForceEstimationConfigurationMessage()
   {
      external_force_position_ = new us.ihmc.euclid.tuple3D.Point3D();
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

      end_effector_hash_code_ = other.end_effector_hash_code_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.external_force_position_, external_force_position_);
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
            * The is the unique hash code of the end-effector that is assumed to be pushed
            * See RigidBody.hashCode() for the computation of a rigid-body hash code.
            */
   public void setEndEffectorHashCode(int end_effector_hash_code)
   {
      end_effector_hash_code_ = end_effector_hash_code;
   }
   /**
            * The is the unique hash code of the end-effector that is assumed to be pushed
            * See RigidBody.hashCode() for the computation of a rigid-body hash code.
            */
   public int getEndEffectorHashCode()
   {
      return end_effector_hash_code_;
   }


   /**
            * This is the position externally applied force to be estimated, in endEffector.getBodyFixedFrame().
            */
   public us.ihmc.euclid.tuple3D.Point3D getExternalForcePosition()
   {
      return external_force_position_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_effector_hash_code_, other.end_effector_hash_code_, epsilon)) return false;

      if (!this.external_force_position_.epsilonEquals(other.external_force_position_, epsilon)) return false;

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

      if(this.end_effector_hash_code_ != otherMyClass.end_effector_hash_code_) return false;

      if (!this.external_force_position_.equals(otherMyClass.external_force_position_)) return false;

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
      builder.append("end_effector_hash_code=");
      builder.append(this.end_effector_hash_code_);      builder.append(", ");
      builder.append("external_force_position=");
      builder.append(this.external_force_position_);
      builder.append("}");
      return builder.toString();
   }
}
