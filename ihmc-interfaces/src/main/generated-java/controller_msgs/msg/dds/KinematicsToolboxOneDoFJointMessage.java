package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * This message contains all the information needed to configure the objectives/constraints to apply on
       * a given 1-DoF joint in the solver.
       */
public class KinematicsToolboxOneDoFJointMessage extends Packet<KinematicsToolboxOneDoFJointMessage> implements Settable<KinematicsToolboxOneDoFJointMessage>, EpsilonComparable<KinematicsToolboxOneDoFJointMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The is the unique hash code of the joint to be solved for.
            * It is used on the solver side to retrieve the desired joint to be controlled.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public int joint_hash_code_;

   /**
            * The is the desired joint position.
            */
   public double desired_position_;

   /**
            * Weight used to define the priority for reaching the desired position.
            */
   public double weight_;

   public KinematicsToolboxOneDoFJointMessage()
   {





   }

   public KinematicsToolboxOneDoFJointMessage(KinematicsToolboxOneDoFJointMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxOneDoFJointMessage other)
   {

      sequence_id_ = other.sequence_id_;


      joint_hash_code_ = other.joint_hash_code_;


      desired_position_ = other.desired_position_;


      weight_ = other.weight_;

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
            * The is the unique hash code of the joint to be solved for.
            * It is used on the solver side to retrieve the desired joint to be controlled.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public void setJointHashCode(int joint_hash_code)
   {
      joint_hash_code_ = joint_hash_code;
   }
   /**
            * The is the unique hash code of the joint to be solved for.
            * It is used on the solver side to retrieve the desired joint to be controlled.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public int getJointHashCode()
   {
      return joint_hash_code_;
   }


   /**
            * The is the desired joint position.
            */
   public void setDesiredPosition(double desired_position)
   {
      desired_position_ = desired_position;
   }
   /**
            * The is the desired joint position.
            */
   public double getDesiredPosition()
   {
      return desired_position_;
   }


   /**
            * Weight used to define the priority for reaching the desired position.
            */
   public void setWeight(double weight)
   {
      weight_ = weight;
   }
   /**
            * Weight used to define the priority for reaching the desired position.
            */
   public double getWeight()
   {
      return weight_;
   }


   public static Supplier<KinematicsToolboxOneDoFJointMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxOneDoFJointMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxOneDoFJointMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxOneDoFJointMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_hash_code_, other.joint_hash_code_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_, other.desired_position_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_, other.weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxOneDoFJointMessage)) return false;

      KinematicsToolboxOneDoFJointMessage otherMyClass = (KinematicsToolboxOneDoFJointMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.joint_hash_code_ != otherMyClass.joint_hash_code_) return false;


      if(this.desired_position_ != otherMyClass.desired_position_) return false;


      if(this.weight_ != otherMyClass.weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxOneDoFJointMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("joint_hash_code=");
      builder.append(this.joint_hash_code_);      builder.append(", ");

      builder.append("desired_position=");
      builder.append(this.desired_position_);      builder.append(", ");

      builder.append("weight=");
      builder.append(this.weight_);
      builder.append("}");
      return builder.toString();
   }
}
