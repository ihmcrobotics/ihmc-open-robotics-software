package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to apply a wrench (force & moment) profile on a hand given a list of trajectory points.
       * A linear interpolation is used to interpolate between trajectory points.
       */
public class HandWrenchTrajectoryMessage extends Packet<HandWrenchTrajectoryMessage> implements Settable<HandWrenchTrajectoryMessage>, EpsilonComparable<HandWrenchTrajectoryMessage>
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
            * Specifies which hand will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * The trajectory information for the force/moment to be achieved by the end-effector.
            */
   public controller_msgs.msg.dds.WrenchTrajectoryMessage wrench_trajectory_;

   public HandWrenchTrajectoryMessage()
   {




      wrench_trajectory_ = new controller_msgs.msg.dds.WrenchTrajectoryMessage();

   }

   public HandWrenchTrajectoryMessage(HandWrenchTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(HandWrenchTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      force_execution_ = other.force_execution_;


      robot_side_ = other.robot_side_;


      controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType.staticCopy(other.wrench_trajectory_, wrench_trajectory_);
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
            * Specifies which hand will execute the trajectory.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which hand will execute the trajectory.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }



   /**
            * The trajectory information for the force/moment to be achieved by the end-effector.
            */
   public controller_msgs.msg.dds.WrenchTrajectoryMessage getWrenchTrajectory()
   {
      return wrench_trajectory_;
   }


   public static Supplier<HandWrenchTrajectoryMessagePubSubType> getPubSubType()
   {
      return HandWrenchTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandWrenchTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandWrenchTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.force_execution_, other.force_execution_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!this.wrench_trajectory_.epsilonEquals(other.wrench_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandWrenchTrajectoryMessage)) return false;

      HandWrenchTrajectoryMessage otherMyClass = (HandWrenchTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.force_execution_ != otherMyClass.force_execution_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.wrench_trajectory_.equals(otherMyClass.wrench_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandWrenchTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("force_execution=");
      builder.append(this.force_execution_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("wrench_trajectory=");
      builder.append(this.wrench_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
