package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the FootstepStreamingToolbox API
       */
public class FootstepStreamingToolboxInputMessage extends Packet<FootstepStreamingToolboxInputMessage> implements Settable<FootstepStreamingToolboxInputMessage>, EpsilonComparable<FootstepStreamingToolboxInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public long sequence_id_;
   /**
            * Current robot step duration
            */
   public double robot_step_duration_;
   /**
            * Time elapsed since the robot has initiated this last step
            */
   public double robot_step_elapsed_time_;
   /**
            * Current swing foot side of the robot
            */
   public byte robot_swing_side_ = (byte) 255;
   /**
            * If the robot swing foot is landing. Foot is descending below max swing height
            */
   public boolean is_robot_swing_foot_landing_;
   /**
            * The list of sides used as input for the toolbox to estimate footsteps for the robot
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage>  side_;

   public FootstepStreamingToolboxInputMessage()
   {
      side_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage> (2, new toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessagePubSubType());

   }

   public FootstepStreamingToolboxInputMessage(FootstepStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_step_duration_ = other.robot_step_duration_;

      robot_step_elapsed_time_ = other.robot_step_elapsed_time_;

      robot_swing_side_ = other.robot_swing_side_;

      is_robot_swing_foot_landing_ = other.is_robot_swing_foot_landing_;

      side_.set(other.side_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Current robot step duration
            */
   public void setRobotStepDuration(double robot_step_duration)
   {
      robot_step_duration_ = robot_step_duration;
   }
   /**
            * Current robot step duration
            */
   public double getRobotStepDuration()
   {
      return robot_step_duration_;
   }

   /**
            * Time elapsed since the robot has initiated this last step
            */
   public void setRobotStepElapsedTime(double robot_step_elapsed_time)
   {
      robot_step_elapsed_time_ = robot_step_elapsed_time;
   }
   /**
            * Time elapsed since the robot has initiated this last step
            */
   public double getRobotStepElapsedTime()
   {
      return robot_step_elapsed_time_;
   }

   /**
            * Current swing foot side of the robot
            */
   public void setRobotSwingSide(byte robot_swing_side)
   {
      robot_swing_side_ = robot_swing_side;
   }
   /**
            * Current swing foot side of the robot
            */
   public byte getRobotSwingSide()
   {
      return robot_swing_side_;
   }

   /**
            * If the robot swing foot is landing. Foot is descending below max swing height
            */
   public void setIsRobotSwingFootLanding(boolean is_robot_swing_foot_landing)
   {
      is_robot_swing_foot_landing_ = is_robot_swing_foot_landing;
   }
   /**
            * If the robot swing foot is landing. Foot is descending below max swing height
            */
   public boolean getIsRobotSwingFootLanding()
   {
      return is_robot_swing_foot_landing_;
   }


   /**
            * The list of sides used as input for the toolbox to estimate footsteps for the robot
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage>  getSide()
   {
      return side_;
   }


   public static Supplier<FootstepStreamingToolboxInputMessagePubSubType> getPubSubType()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepStreamingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_step_duration_, other.robot_step_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_step_elapsed_time_, other.robot_step_elapsed_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_swing_side_, other.robot_swing_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_robot_swing_foot_landing_, other.is_robot_swing_foot_landing_, epsilon)) return false;

      if (this.side_.size() != other.side_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.side_.size(); i++)
         {  if (!this.side_.get(i).epsilonEquals(other.side_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepStreamingToolboxInputMessage)) return false;

      FootstepStreamingToolboxInputMessage otherMyClass = (FootstepStreamingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_step_duration_ != otherMyClass.robot_step_duration_) return false;

      if(this.robot_step_elapsed_time_ != otherMyClass.robot_step_elapsed_time_) return false;

      if(this.robot_swing_side_ != otherMyClass.robot_swing_side_) return false;

      if(this.is_robot_swing_foot_landing_ != otherMyClass.is_robot_swing_foot_landing_) return false;

      if (!this.side_.equals(otherMyClass.side_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_step_duration=");
      builder.append(this.robot_step_duration_);      builder.append(", ");
      builder.append("robot_step_elapsed_time=");
      builder.append(this.robot_step_elapsed_time_);      builder.append(", ");
      builder.append("robot_swing_side=");
      builder.append(this.robot_swing_side_);      builder.append(", ");
      builder.append("is_robot_swing_foot_landing=");
      builder.append(this.is_robot_swing_foot_landing_);      builder.append(", ");
      builder.append("side=");
      builder.append(this.side_);
      builder.append("}");
      return builder.toString();
   }
}
