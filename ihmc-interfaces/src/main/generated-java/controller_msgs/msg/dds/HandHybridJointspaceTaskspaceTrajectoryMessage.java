package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move the hand in both taskspace and jointspace
       * to the desired orientation and joint angles while going through the specified trajectory points.
       */
public class HandHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HandHybridJointspaceTaskspaceTrajectoryMessage> implements Settable<HandHybridJointspaceTaskspaceTrajectoryMessage>, EpsilonComparable<HandHybridJointspaceTaskspaceTrajectoryMessage>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public boolean force_execution_;

   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * The taskspace trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage taskspace_trajectory_message_;

   /**
            * The jointspace trajectory information.
            * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_message_;

   public HandHybridJointspaceTaskspaceTrajectoryMessage()
   {




      taskspace_trajectory_message_ = new controller_msgs.msg.dds.SE3TrajectoryMessage();

      jointspace_trajectory_message_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();

   }

   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandHybridJointspaceTaskspaceTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(HandHybridJointspaceTaskspaceTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      force_execution_ = other.force_execution_;


      robot_side_ = other.robot_side_;


      controller_msgs.msg.dds.SE3TrajectoryMessagePubSubType.staticCopy(other.taskspace_trajectory_message_, taskspace_trajectory_message_);

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_message_, jointspace_trajectory_message_);
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
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public void setForceExecution(boolean force_execution)
   {
      force_execution_ = force_execution;
   }
   /**
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public boolean getForceExecution()
   {
      return force_execution_;
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
            * The taskspace trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage getTaskspaceTrajectoryMessage()
   {
      return taskspace_trajectory_message_;
   }



   /**
            * The jointspace trajectory information.
            * The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectoryMessage()
   {
      return jointspace_trajectory_message_;
   }


   public static Supplier<HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType> getPubSubType()
   {
      return HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.force_execution_, other.force_execution_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!this.taskspace_trajectory_message_.epsilonEquals(other.taskspace_trajectory_message_, epsilon)) return false;

      if (!this.jointspace_trajectory_message_.epsilonEquals(other.jointspace_trajectory_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandHybridJointspaceTaskspaceTrajectoryMessage)) return false;

      HandHybridJointspaceTaskspaceTrajectoryMessage otherMyClass = (HandHybridJointspaceTaskspaceTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.force_execution_ != otherMyClass.force_execution_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.taskspace_trajectory_message_.equals(otherMyClass.taskspace_trajectory_message_)) return false;

      if (!this.jointspace_trajectory_message_.equals(otherMyClass.jointspace_trajectory_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandHybridJointspaceTaskspaceTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("force_execution=");
      builder.append(this.force_execution_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("taskspace_trajectory_message=");
      builder.append(this.taskspace_trajectory_message_);      builder.append(", ");

      builder.append("jointspace_trajectory_message=");
      builder.append(this.jointspace_trajectory_message_);
      builder.append("}");
      return builder.toString();
   }
}
