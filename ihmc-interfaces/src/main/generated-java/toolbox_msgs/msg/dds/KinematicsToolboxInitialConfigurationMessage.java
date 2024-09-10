package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * It contains auxiliary information that allows to customize the initial configuration.
       */
public class KinematicsToolboxInitialConfigurationMessage extends Packet<KinematicsToolboxInitialConfigurationMessage> implements Settable<KinematicsToolboxInitialConfigurationMessage>, EpsilonComparable<KinematicsToolboxInitialConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * This array is used to identify to which joint each angle in initial_joint_angles belongs to.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  initial_joint_hash_codes_;
   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.idl.IDLSequence.Float  initial_joint_angles_;

   public KinematicsToolboxInitialConfigurationMessage()
   {
      initial_joint_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      initial_joint_angles_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public KinematicsToolboxInitialConfigurationMessage(KinematicsToolboxInitialConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxInitialConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      initial_joint_hash_codes_.set(other.initial_joint_hash_codes_);
      initial_joint_angles_.set(other.initial_joint_angles_);
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
            * This array is used to identify to which joint each angle in initial_joint_angles belongs to.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  getInitialJointHashCodes()
   {
      return initial_joint_hash_codes_;
   }


   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.idl.IDLSequence.Float  getInitialJointAngles()
   {
      return initial_joint_angles_;
   }


   public static Supplier<KinematicsToolboxInitialConfigurationMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxInitialConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxInitialConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxInitialConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.initial_joint_hash_codes_, other.initial_joint_hash_codes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.initial_joint_angles_, other.initial_joint_angles_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxInitialConfigurationMessage)) return false;

      KinematicsToolboxInitialConfigurationMessage otherMyClass = (KinematicsToolboxInitialConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.initial_joint_hash_codes_.equals(otherMyClass.initial_joint_hash_codes_)) return false;
      if (!this.initial_joint_angles_.equals(otherMyClass.initial_joint_angles_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxInitialConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("initial_joint_hash_codes=");
      builder.append(this.initial_joint_hash_codes_);      builder.append(", ");
      builder.append("initial_joint_angles=");
      builder.append(this.initial_joint_angles_);
      builder.append("}");
      return builder.toString();
   }
}
