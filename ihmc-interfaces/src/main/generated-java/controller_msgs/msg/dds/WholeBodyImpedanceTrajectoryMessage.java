package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the whole-body impedance controller API.
       * Allows to send joint-space trajectory along with a timed contact sequence which determines limb stiffness.
       */
public class WholeBodyImpedanceTrajectoryMessage extends Packet<WholeBodyImpedanceTrajectoryMessage> implements Settable<WholeBodyImpedanceTrajectoryMessage>, EpsilonComparable<WholeBodyImpedanceTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The is the unique hash code of each of the joints to be controlled.
            * It is used on the controller side to retrieve the desired joint to be controlled.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  joint_hash_codes_;
   /**
            * Trajectory for each joint.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  joint_trajectory_messages_;
   /**
            * Properties for queueing trajectories.
            */
   public ihmc_common_msgs.msg.dds.QueueableMessage queueing_properties_;
   /**
            * Contact sequence message
            */
   public controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage contact_sequence_message_;

   public WholeBodyImpedanceTrajectoryMessage()
   {
      joint_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      joint_trajectory_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage> (100, new controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType());
      queueing_properties_ = new ihmc_common_msgs.msg.dds.QueueableMessage();
      contact_sequence_message_ = new controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage();

   }

   public WholeBodyImpedanceTrajectoryMessage(WholeBodyImpedanceTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyImpedanceTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_hash_codes_.set(other.joint_hash_codes_);
      joint_trajectory_messages_.set(other.joint_trajectory_messages_);
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
      controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType.staticCopy(other.contact_sequence_message_, contact_sequence_message_);
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
            * The is the unique hash code of each of the joints to be controlled.
            * It is used on the controller side to retrieve the desired joint to be controlled.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  getJointHashCodes()
   {
      return joint_hash_codes_;
   }


   /**
            * Trajectory for each joint.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  getJointTrajectoryMessages()
   {
      return joint_trajectory_messages_;
   }


   /**
            * Properties for queueing trajectories.
            */
   public ihmc_common_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   /**
            * Contact sequence message
            */
   public controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage getContactSequenceMessage()
   {
      return contact_sequence_message_;
   }


   public static Supplier<WholeBodyImpedanceTrajectoryMessagePubSubType> getPubSubType()
   {
      return WholeBodyImpedanceTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyImpedanceTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyImpedanceTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.joint_hash_codes_, other.joint_hash_codes_, epsilon)) return false;

      if (this.joint_trajectory_messages_.size() != other.joint_trajectory_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.joint_trajectory_messages_.size(); i++)
         {  if (!this.joint_trajectory_messages_.get(i).epsilonEquals(other.joint_trajectory_messages_.get(i), epsilon)) return false; }
      }

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;
      if (!this.contact_sequence_message_.epsilonEquals(other.contact_sequence_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyImpedanceTrajectoryMessage)) return false;

      WholeBodyImpedanceTrajectoryMessage otherMyClass = (WholeBodyImpedanceTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.joint_hash_codes_.equals(otherMyClass.joint_hash_codes_)) return false;
      if (!this.joint_trajectory_messages_.equals(otherMyClass.joint_trajectory_messages_)) return false;
      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;
      if (!this.contact_sequence_message_.equals(otherMyClass.contact_sequence_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyImpedanceTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_hash_codes=");
      builder.append(this.joint_hash_codes_);      builder.append(", ");
      builder.append("joint_trajectory_messages=");
      builder.append(this.joint_trajectory_messages_);      builder.append(", ");
      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);      builder.append(", ");
      builder.append("contact_sequence_message=");
      builder.append(this.contact_sequence_message_);
      builder.append("}");
      return builder.toString();
   }
}
