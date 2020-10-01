package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       * Experimental mode of specifying a desired configuration in terms of root pose and joint angles.
       */
public class WholeBodyMultiContactTrajectoryMessage extends Packet<WholeBodyMultiContactTrajectoryMessage> implements Settable<WholeBodyMultiContactTrajectoryMessage>, EpsilonComparable<WholeBodyMultiContactTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Duration of trajectory in seconds
            */
   public double trajectory_duration_;
   /**
            * Target joint angles of the trajectory
            */
   public us.ihmc.idl.IDLSequence.Double  joint_angles_;
   /**
            * Terminal pelvis pose in world frame
            */
   public us.ihmc.euclid.geometry.Pose3D pelvis_pose_;
   /**
            * Hash of joint array
            */
   public int joint_name_hash_;
   /**
            * Contact state changes associated with this trajectory
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ContactStateChangeMessage>  contact_state_changes_;

   public WholeBodyMultiContactTrajectoryMessage()
   {
      joint_angles_ = new us.ihmc.idl.IDLSequence.Double (50, "type_6");

      pelvis_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      contact_state_changes_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ContactStateChangeMessage> (10, new controller_msgs.msg.dds.ContactStateChangeMessagePubSubType());

   }

   public WholeBodyMultiContactTrajectoryMessage(WholeBodyMultiContactTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyMultiContactTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      trajectory_duration_ = other.trajectory_duration_;

      joint_angles_.set(other.joint_angles_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pelvis_pose_, pelvis_pose_);
      joint_name_hash_ = other.joint_name_hash_;

      contact_state_changes_.set(other.contact_state_changes_);
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
            * Duration of trajectory in seconds
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * Duration of trajectory in seconds
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   /**
            * Target joint angles of the trajectory
            */
   public us.ihmc.idl.IDLSequence.Double  getJointAngles()
   {
      return joint_angles_;
   }


   /**
            * Terminal pelvis pose in world frame
            */
   public us.ihmc.euclid.geometry.Pose3D getPelvisPose()
   {
      return pelvis_pose_;
   }

   /**
            * Hash of joint array
            */
   public void setJointNameHash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }
   /**
            * Hash of joint array
            */
   public int getJointNameHash()
   {
      return joint_name_hash_;
   }


   /**
            * Contact state changes associated with this trajectory
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ContactStateChangeMessage>  getContactStateChanges()
   {
      return contact_state_changes_;
   }


   public static Supplier<WholeBodyMultiContactTrajectoryMessagePubSubType> getPubSubType()
   {
      return WholeBodyMultiContactTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyMultiContactTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyMultiContactTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.joint_angles_, other.joint_angles_, epsilon)) return false;

      if (!this.pelvis_pose_.epsilonEquals(other.pelvis_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon)) return false;

      if (this.contact_state_changes_.size() != other.contact_state_changes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_state_changes_.size(); i++)
         {  if (!this.contact_state_changes_.get(i).epsilonEquals(other.contact_state_changes_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyMultiContactTrajectoryMessage)) return false;

      WholeBodyMultiContactTrajectoryMessage otherMyClass = (WholeBodyMultiContactTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if (!this.joint_angles_.equals(otherMyClass.joint_angles_)) return false;
      if (!this.pelvis_pose_.equals(otherMyClass.pelvis_pose_)) return false;
      if(this.joint_name_hash_ != otherMyClass.joint_name_hash_) return false;

      if (!this.contact_state_changes_.equals(otherMyClass.contact_state_changes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyMultiContactTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(this.joint_angles_);      builder.append(", ");
      builder.append("pelvis_pose=");
      builder.append(this.pelvis_pose_);      builder.append(", ");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);      builder.append(", ");
      builder.append("contact_state_changes=");
      builder.append(this.contact_state_changes_);
      builder.append("}");
      return builder.toString();
   }
}
