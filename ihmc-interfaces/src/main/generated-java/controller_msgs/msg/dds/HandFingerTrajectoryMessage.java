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
            * The indexing for the joints is defined in "ValkyrieFingerMotorName".
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_;

   public HandFingerTrajectoryMessage()
   {
      jointspace_trajectory_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();
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

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_, jointspace_trajectory_);
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
            * The indexing for the joints is defined in "ValkyrieFingerMotorName".
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspace_trajectory_;
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

      if (!this.jointspace_trajectory_.epsilonEquals(other.jointspace_trajectory_, epsilon)) return false;

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

      if (!this.jointspace_trajectory_.equals(otherMyClass.jointspace_trajectory_)) return false;

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
      builder.append("jointspace_trajectory=");
      builder.append(this.jointspace_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
