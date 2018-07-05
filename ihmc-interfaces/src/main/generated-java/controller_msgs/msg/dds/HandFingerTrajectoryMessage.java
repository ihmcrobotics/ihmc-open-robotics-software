package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move fingers in motor control space.
       */
public class HandFingerTrajectoryMessage extends Packet<HandFingerTrajectoryMessage> implements Settable<HandFingerTrajectoryMessage>, EpsilonComparable<HandFingerTrajectoryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Trajectories for each joint(finger motors).
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  joint_trajectory_messages_;
   /**
            * Properties for queueing trajectories.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueueableMessage>  list_queueing_properties_;

   public HandFingerTrajectoryMessage()
   {
      joint_trajectory_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage> (100, new controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType());
      list_queueing_properties_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueueableMessage> (100, new controller_msgs.msg.dds.QueueableMessagePubSubType());

   }

   public HandFingerTrajectoryMessage(HandFingerTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(HandFingerTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      joint_trajectory_messages_.set(other.joint_trajectory_messages_);
      list_queueing_properties_.set(other.list_queueing_properties_);
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
            * Specifies the side of the robot that will execute the trajectory.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Trajectories for each joint(finger motors).
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.OneDoFJointTrajectoryMessage>  getJointTrajectoryMessages()
   {
      return joint_trajectory_messages_;
   }


   /**
            * Properties for queueing trajectories.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.QueueableMessage>  getListQueueingProperties()
   {
      return list_queueing_properties_;
   }


   public static Supplier<HandFingerTrajectoryMessagePubSubType> getPubSubType()
   {
      return HandFingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandFingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandFingerTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (this.joint_trajectory_messages_.size() != other.joint_trajectory_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.joint_trajectory_messages_.size(); i++)
         {  if (!this.joint_trajectory_messages_.get(i).epsilonEquals(other.joint_trajectory_messages_.get(i), epsilon)) return false; }
      }

      if (this.list_queueing_properties_.size() != other.list_queueing_properties_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.list_queueing_properties_.size(); i++)
         {  if (!this.list_queueing_properties_.get(i).epsilonEquals(other.list_queueing_properties_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandFingerTrajectoryMessage)) return false;

      HandFingerTrajectoryMessage otherMyClass = (HandFingerTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.joint_trajectory_messages_.equals(otherMyClass.joint_trajectory_messages_)) return false;
      if (!this.list_queueing_properties_.equals(otherMyClass.list_queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandFingerTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("joint_trajectory_messages=");
      builder.append(this.joint_trajectory_messages_);      builder.append(", ");
      builder.append("list_queueing_properties=");
      builder.append(this.list_queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
