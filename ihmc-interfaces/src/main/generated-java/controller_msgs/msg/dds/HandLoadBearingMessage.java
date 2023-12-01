package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Request the controller to use a hand to help supporting the robot weight.
       */
public class HandLoadBearingMessage extends Packet<HandLoadBearingMessage> implements Settable<HandLoadBearingMessage>, EpsilonComparable<HandLoadBearingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The robot side of the hand that will be load bearing.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Determines the control mode used for the arm's nullspace. If true, jointspace is used, otherwise taskspace orientation is used.
            */
   public boolean use_jointspace_command_ = true;
   /**
            * The arm desired jointspace trajectory that will be used to control the arm's nullspace if use_jointspace_command is true.
            * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_;
   /**
            * The hand desired orientation trajectory that will be used to control the arm's nullspace if use_jointspace_command is false.
            */
   public ihmc_common_msgs.msg.dds.SO3TrajectoryMessage orientation_trajectory_;
   /**
            * The time to delay this message on the controller side before being executed.
            */
   public double execution_delay_time_;
   /**
            * Information specific to the load bearing properties.
            */
   public controller_msgs.msg.dds.LoadBearingMessage load_bearing_message_;

   public HandLoadBearingMessage()
   {
      jointspace_trajectory_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();
      orientation_trajectory_ = new ihmc_common_msgs.msg.dds.SO3TrajectoryMessage();
      load_bearing_message_ = new controller_msgs.msg.dds.LoadBearingMessage();
   }

   public HandLoadBearingMessage(HandLoadBearingMessage other)
   {
      this();
      set(other);
   }

   public void set(HandLoadBearingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      use_jointspace_command_ = other.use_jointspace_command_;

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_, jointspace_trajectory_);
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.orientation_trajectory_, orientation_trajectory_);
      execution_delay_time_ = other.execution_delay_time_;

      controller_msgs.msg.dds.LoadBearingMessagePubSubType.staticCopy(other.load_bearing_message_, load_bearing_message_);
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
            * The robot side of the hand that will be load bearing.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * The robot side of the hand that will be load bearing.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Determines the control mode used for the arm's nullspace. If true, jointspace is used, otherwise taskspace orientation is used.
            */
   public void setUseJointspaceCommand(boolean use_jointspace_command)
   {
      use_jointspace_command_ = use_jointspace_command;
   }
   /**
            * Determines the control mode used for the arm's nullspace. If true, jointspace is used, otherwise taskspace orientation is used.
            */
   public boolean getUseJointspaceCommand()
   {
      return use_jointspace_command_;
   }


   /**
            * The arm desired jointspace trajectory that will be used to control the arm's nullspace if use_jointspace_command is true.
            * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspace_trajectory_;
   }


   /**
            * The hand desired orientation trajectory that will be used to control the arm's nullspace if use_jointspace_command is false.
            */
   public ihmc_common_msgs.msg.dds.SO3TrajectoryMessage getOrientationTrajectory()
   {
      return orientation_trajectory_;
   }

   /**
            * The time to delay this message on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this message on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }


   /**
            * Information specific to the load bearing properties.
            */
   public controller_msgs.msg.dds.LoadBearingMessage getLoadBearingMessage()
   {
      return load_bearing_message_;
   }


   public static Supplier<HandLoadBearingMessagePubSubType> getPubSubType()
   {
      return HandLoadBearingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandLoadBearingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandLoadBearingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_jointspace_command_, other.use_jointspace_command_, epsilon)) return false;

      if (!this.jointspace_trajectory_.epsilonEquals(other.jointspace_trajectory_, epsilon)) return false;
      if (!this.orientation_trajectory_.epsilonEquals(other.orientation_trajectory_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;

      if (!this.load_bearing_message_.epsilonEquals(other.load_bearing_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandLoadBearingMessage)) return false;

      HandLoadBearingMessage otherMyClass = (HandLoadBearingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.use_jointspace_command_ != otherMyClass.use_jointspace_command_) return false;

      if (!this.jointspace_trajectory_.equals(otherMyClass.jointspace_trajectory_)) return false;
      if (!this.orientation_trajectory_.equals(otherMyClass.orientation_trajectory_)) return false;
      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;

      if (!this.load_bearing_message_.equals(otherMyClass.load_bearing_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandLoadBearingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("use_jointspace_command=");
      builder.append(this.use_jointspace_command_);      builder.append(", ");
      builder.append("jointspace_trajectory=");
      builder.append(this.jointspace_trajectory_);      builder.append(", ");
      builder.append("orientation_trajectory=");
      builder.append(this.orientation_trajectory_);      builder.append(", ");
      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);      builder.append(", ");
      builder.append("load_bearing_message=");
      builder.append(this.load_bearing_message_);
      builder.append("}");
      return builder.toString();
   }
}
