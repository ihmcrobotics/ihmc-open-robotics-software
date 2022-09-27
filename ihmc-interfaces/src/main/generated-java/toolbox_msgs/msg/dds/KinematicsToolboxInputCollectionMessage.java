package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * This message allows to pack and send as a single message a collection of inputs for the kinematics toolbox.
       */
public class KinematicsToolboxInputCollectionMessage extends Packet<KinematicsToolboxInputCollectionMessage> implements Settable<KinematicsToolboxInputCollectionMessage>, EpsilonComparable<KinematicsToolboxInputCollectionMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Inputs for controlling the center of mass position.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage>  center_of_mass_inputs_;
   /**
            * Inputs for controlling rigid-bodies.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  rigid_body_inputs_;
   /**
            * Inputs for controlling 1-DoF joints.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage>  joint_inputs_;
   /**
            * Input for overriding the default support polygon with a custom one.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage>  contact_state_input_;

   public KinematicsToolboxInputCollectionMessage()
   {
      center_of_mass_inputs_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage> (3, new toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType());
      rigid_body_inputs_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage> (20, new toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType());
      joint_inputs_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage> (20, new toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessagePubSubType());
      contact_state_input_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage> (1, new toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessagePubSubType());

   }

   public KinematicsToolboxInputCollectionMessage(KinematicsToolboxInputCollectionMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxInputCollectionMessage other)
   {
      sequence_id_ = other.sequence_id_;

      center_of_mass_inputs_.set(other.center_of_mass_inputs_);
      rigid_body_inputs_.set(other.rigid_body_inputs_);
      joint_inputs_.set(other.joint_inputs_);
      contact_state_input_.set(other.contact_state_input_);
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
            * Inputs for controlling the center of mass position.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage>  getCenterOfMassInputs()
   {
      return center_of_mass_inputs_;
   }


   /**
            * Inputs for controlling rigid-bodies.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  getRigidBodyInputs()
   {
      return rigid_body_inputs_;
   }


   /**
            * Inputs for controlling 1-DoF joints.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage>  getJointInputs()
   {
      return joint_inputs_;
   }


   /**
            * Input for overriding the default support polygon with a custom one.
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage>  getContactStateInput()
   {
      return contact_state_input_;
   }


   public static Supplier<KinematicsToolboxInputCollectionMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxInputCollectionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxInputCollectionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxInputCollectionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.center_of_mass_inputs_.size() != other.center_of_mass_inputs_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.center_of_mass_inputs_.size(); i++)
         {  if (!this.center_of_mass_inputs_.get(i).epsilonEquals(other.center_of_mass_inputs_.get(i), epsilon)) return false; }
      }

      if (this.rigid_body_inputs_.size() != other.rigid_body_inputs_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.rigid_body_inputs_.size(); i++)
         {  if (!this.rigid_body_inputs_.get(i).epsilonEquals(other.rigid_body_inputs_.get(i), epsilon)) return false; }
      }

      if (this.joint_inputs_.size() != other.joint_inputs_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.joint_inputs_.size(); i++)
         {  if (!this.joint_inputs_.get(i).epsilonEquals(other.joint_inputs_.get(i), epsilon)) return false; }
      }

      if (this.contact_state_input_.size() != other.contact_state_input_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_state_input_.size(); i++)
         {  if (!this.contact_state_input_.get(i).epsilonEquals(other.contact_state_input_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxInputCollectionMessage)) return false;

      KinematicsToolboxInputCollectionMessage otherMyClass = (KinematicsToolboxInputCollectionMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.center_of_mass_inputs_.equals(otherMyClass.center_of_mass_inputs_)) return false;
      if (!this.rigid_body_inputs_.equals(otherMyClass.rigid_body_inputs_)) return false;
      if (!this.joint_inputs_.equals(otherMyClass.joint_inputs_)) return false;
      if (!this.contact_state_input_.equals(otherMyClass.contact_state_input_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxInputCollectionMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("center_of_mass_inputs=");
      builder.append(this.center_of_mass_inputs_);      builder.append(", ");
      builder.append("rigid_body_inputs=");
      builder.append(this.rigid_body_inputs_);      builder.append(", ");
      builder.append("joint_inputs=");
      builder.append(this.joint_inputs_);      builder.append(", ");
      builder.append("contact_state_input=");
      builder.append(this.contact_state_input_);
      builder.append("}");
      return builder.toString();
   }
}
