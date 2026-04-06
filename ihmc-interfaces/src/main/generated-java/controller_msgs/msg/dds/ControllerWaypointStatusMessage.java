package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Published by the PDVelocityBasedGoalReacher when it begins walking toward a waypoint
       * or when it reaches a waypoint.
       */
public class ControllerWaypointStatusMessage extends Packet<ControllerWaypointStatusMessage> implements Settable<ControllerWaypointStatusMessage>, EpsilonComparable<ControllerWaypointStatusMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The waypoint goal pose associated with this event.
            */
   public double goal_x_position_;
   public double goal_y_position_;
   public double goal_yaw_;
   /**
            * Number of waypoints remaining in the queue after this event (not counting the waypoint
            * described by this message).
            */
   public int waypoints_remaining_;
   /**
            * True if the robot just started walking toward this waypoint.
            * False if the robot just reached this waypoint.
            */
   public boolean is_started_;

   public ControllerWaypointStatusMessage()
   {
   }

   public ControllerWaypointStatusMessage(ControllerWaypointStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ControllerWaypointStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      goal_x_position_ = other.goal_x_position_;

      goal_y_position_ = other.goal_y_position_;

      goal_yaw_ = other.goal_yaw_;

      waypoints_remaining_ = other.waypoints_remaining_;

      is_started_ = other.is_started_;

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
            * The waypoint goal pose associated with this event.
            */
   public void setGoalXPosition(double goal_x_position)
   {
      goal_x_position_ = goal_x_position;
   }
   /**
            * The waypoint goal pose associated with this event.
            */
   public double getGoalXPosition()
   {
      return goal_x_position_;
   }

   public void setGoalYPosition(double goal_y_position)
   {
      goal_y_position_ = goal_y_position;
   }
   public double getGoalYPosition()
   {
      return goal_y_position_;
   }

   public void setGoalYaw(double goal_yaw)
   {
      goal_yaw_ = goal_yaw;
   }
   public double getGoalYaw()
   {
      return goal_yaw_;
   }

   /**
            * Number of waypoints remaining in the queue after this event (not counting the waypoint
            * described by this message).
            */
   public void setWaypointsRemaining(int waypoints_remaining)
   {
      waypoints_remaining_ = waypoints_remaining;
   }
   /**
            * Number of waypoints remaining in the queue after this event (not counting the waypoint
            * described by this message).
            */
   public int getWaypointsRemaining()
   {
      return waypoints_remaining_;
   }

   /**
            * True if the robot just started walking toward this waypoint.
            * False if the robot just reached this waypoint.
            */
   public void setIsStarted(boolean is_started)
   {
      is_started_ = is_started;
   }
   /**
            * True if the robot just started walking toward this waypoint.
            * False if the robot just reached this waypoint.
            */
   public boolean getIsStarted()
   {
      return is_started_;
   }


   public static Supplier<ControllerWaypointStatusMessagePubSubType> getPubSubType()
   {
      return ControllerWaypointStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerWaypointStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerWaypointStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_x_position_, other.goal_x_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_y_position_, other.goal_y_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_yaw_, other.goal_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.waypoints_remaining_, other.waypoints_remaining_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_started_, other.is_started_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerWaypointStatusMessage)) return false;

      ControllerWaypointStatusMessage otherMyClass = (ControllerWaypointStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.goal_x_position_ != otherMyClass.goal_x_position_) return false;

      if(this.goal_y_position_ != otherMyClass.goal_y_position_) return false;

      if(this.goal_yaw_ != otherMyClass.goal_yaw_) return false;

      if(this.waypoints_remaining_ != otherMyClass.waypoints_remaining_) return false;

      if(this.is_started_ != otherMyClass.is_started_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerWaypointStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("goal_x_position=");
      builder.append(this.goal_x_position_);      builder.append(", ");
      builder.append("goal_y_position=");
      builder.append(this.goal_y_position_);      builder.append(", ");
      builder.append("goal_yaw=");
      builder.append(this.goal_yaw_);      builder.append(", ");
      builder.append("waypoints_remaining=");
      builder.append(this.waypoints_remaining_);      builder.append(", ");
      builder.append("is_started=");
      builder.append(this.is_started_);
      builder.append("}");
      return builder.toString();
   }
}
