package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * General purpose message that holds onto information to execute a trajectory in jointspace.
       * A third order polynomial function is used to interpolate between trajectory points.
       * The joint_trajectory_messages can have different waypoint times and different number of waypoints.
       * If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories.
       */
public class JointspaceTrajectoryMessage extends Packet<JointspaceTrajectoryMessage> implements Settable<JointspaceTrajectoryMessage>, EpsilonComparable<JointspaceTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Trajectory for each joint.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  joint_trajectory_messages_;
   /**
            * Properties for queueing trajectories.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;

   public JointspaceTrajectoryMessage()
   {
      joint_trajectory_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage> (100, controller_msgs.msg.dds.OneDoFJointTrajectoryMessage.class, new controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType());
      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();

   }

   public JointspaceTrajectoryMessage(JointspaceTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(JointspaceTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_trajectory_messages_.set(other.joint_trajectory_messages_);
      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
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
            * Trajectory for each joint.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  getJointTrajectoryMessages()
   {
      return joint_trajectory_messages_;
   }


   /**
            * Properties for queueing trajectories.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public static Supplier<JointspaceTrajectoryMessagePubSubType> getPubSubType()
   {
      return JointspaceTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointspaceTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointspaceTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.joint_trajectory_messages_.size() != other.joint_trajectory_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.joint_trajectory_messages_.size(); i++)
         {  if (!this.joint_trajectory_messages_.get(i).epsilonEquals(other.joint_trajectory_messages_.get(i), epsilon)) return false; }
      }

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointspaceTrajectoryMessage)) return false;

      JointspaceTrajectoryMessage otherMyClass = (JointspaceTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.joint_trajectory_messages_.equals(otherMyClass.joint_trajectory_messages_)) return false;
      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointspaceTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_trajectory_messages=");
      builder.append(this.joint_trajectory_messages_);      builder.append(", ");
      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
