package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The message commands the controller to bring the given part of the body back to a default configuration called 'home'.
       * It is useful to get back to a safe configuration before walking.
       */
public class GoHomeMessage extends Packet<GoHomeMessage> implements Settable<GoHomeMessage>, EpsilonComparable<GoHomeMessage>
{

   public static final byte HUMANOID_BODY_PART_ARM = (byte) 0;

   public static final byte HUMANOID_BODY_PART_CHEST = (byte) 1;

   public static final byte HUMANOID_BODY_PART_PELVIS = (byte) 2;

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies the part of the body the use wants to move back to its home configuration.
            */
   public byte humanoid_body_part_ = (byte) 255;

   /**
            * Needed to identify a side dependent end-effector.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * How long the trajectory will spline from the current desired to the home configuration.
            */
   public double trajectory_time_;

   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double execution_delay_time_;

   public GoHomeMessage()
   {






   }

   public GoHomeMessage(GoHomeMessage other)
   {
      this();
      set(other);
   }

   public void set(GoHomeMessage other)
   {

      sequence_id_ = other.sequence_id_;


      humanoid_body_part_ = other.humanoid_body_part_;


      robot_side_ = other.robot_side_;


      trajectory_time_ = other.trajectory_time_;


      execution_delay_time_ = other.execution_delay_time_;

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
            * Specifies the part of the body the use wants to move back to its home configuration.
            */
   public void setHumanoidBodyPart(byte humanoid_body_part)
   {
      humanoid_body_part_ = humanoid_body_part;
   }
   /**
            * Specifies the part of the body the use wants to move back to its home configuration.
            */
   public byte getHumanoidBodyPart()
   {
      return humanoid_body_part_;
   }


   /**
            * Needed to identify a side dependent end-effector.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Needed to identify a side dependent end-effector.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * How long the trajectory will spline from the current desired to the home configuration.
            */
   public void setTrajectoryTime(double trajectory_time)
   {
      trajectory_time_ = trajectory_time;
   }
   /**
            * How long the trajectory will spline from the current desired to the home configuration.
            */
   public double getTrajectoryTime()
   {
      return trajectory_time_;
   }


   /**
            * The time to delay this command on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }


   public static Supplier<GoHomeMessagePubSubType> getPubSubType()
   {
      return GoHomeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GoHomeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GoHomeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.humanoid_body_part_, other.humanoid_body_part_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_time_, other.trajectory_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GoHomeMessage)) return false;

      GoHomeMessage otherMyClass = (GoHomeMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.humanoid_body_part_ != otherMyClass.humanoid_body_part_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if(this.trajectory_time_ != otherMyClass.trajectory_time_) return false;


      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GoHomeMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("humanoid_body_part=");
      builder.append(this.humanoid_body_part_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("trajectory_time=");
      builder.append(this.trajectory_time_);      builder.append(", ");

      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);
      builder.append("}");
      return builder.toString();
   }
}
