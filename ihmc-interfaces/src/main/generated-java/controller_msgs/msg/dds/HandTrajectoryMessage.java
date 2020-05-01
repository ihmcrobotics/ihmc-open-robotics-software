package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points.
       * A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations.
       * To execute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       */
public class HandTrajectoryMessage extends Packet<HandTrajectoryMessage> implements Settable<HandTrajectoryMessage>, EpsilonComparable<HandTrajectoryMessage>
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
            * The position/orientation trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage se3_trajectory_;

   public HandTrajectoryMessage()
   {




      se3_trajectory_ = new controller_msgs.msg.dds.SE3TrajectoryMessage();

   }

   public HandTrajectoryMessage(HandTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(HandTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      force_execution_ = other.force_execution_;


      robot_side_ = other.robot_side_;


      controller_msgs.msg.dds.SE3TrajectoryMessagePubSubType.staticCopy(other.se3_trajectory_, se3_trajectory_);
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
            * The position/orientation trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage getSe3Trajectory()
   {
      return se3_trajectory_;
   }


   public static Supplier<HandTrajectoryMessagePubSubType> getPubSubType()
   {
      return HandTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.force_execution_, other.force_execution_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!this.se3_trajectory_.epsilonEquals(other.se3_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandTrajectoryMessage)) return false;

      HandTrajectoryMessage otherMyClass = (HandTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.force_execution_ != otherMyClass.force_execution_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.se3_trajectory_.equals(otherMyClass.se3_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("force_execution=");
      builder.append(this.force_execution_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("se3_trajectory=");
      builder.append(this.se3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
