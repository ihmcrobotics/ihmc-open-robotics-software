package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Status message for the PDVelocityBasedGoalReacher.
       */
public class ControllerWalkToGoalStatusMessage extends Packet<ControllerWalkToGoalStatusMessage> implements Settable<ControllerWalkToGoalStatusMessage>, EpsilonComparable<ControllerWalkToGoalStatusMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Current pose for the robot to reach.
            */
   public double current_goal_x_position_;
   public double current_goal_y_position_;
   public double current_goal_yaw_;
   public us.ihmc.euclid.tuple3D.Point3D current_position_;
   public boolean is_reached_;

   public ControllerWalkToGoalStatusMessage()
   {
      current_position_ = new us.ihmc.euclid.tuple3D.Point3D();
   }

   public ControllerWalkToGoalStatusMessage(ControllerWalkToGoalStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ControllerWalkToGoalStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      current_goal_x_position_ = other.current_goal_x_position_;

      current_goal_y_position_ = other.current_goal_y_position_;

      current_goal_yaw_ = other.current_goal_yaw_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.current_position_, current_position_);
      is_reached_ = other.is_reached_;

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
            * Current pose for the robot to reach.
            */
   public void setCurrentGoalXPosition(double current_goal_x_position)
   {
      current_goal_x_position_ = current_goal_x_position;
   }
   /**
            * Current pose for the robot to reach.
            */
   public double getCurrentGoalXPosition()
   {
      return current_goal_x_position_;
   }

   public void setCurrentGoalYPosition(double current_goal_y_position)
   {
      current_goal_y_position_ = current_goal_y_position;
   }
   public double getCurrentGoalYPosition()
   {
      return current_goal_y_position_;
   }

   public void setCurrentGoalYaw(double current_goal_yaw)
   {
      current_goal_yaw_ = current_goal_yaw;
   }
   public double getCurrentGoalYaw()
   {
      return current_goal_yaw_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getCurrentPosition()
   {
      return current_position_;
   }

   public void setIsReached(boolean is_reached)
   {
      is_reached_ = is_reached;
   }
   public boolean getIsReached()
   {
      return is_reached_;
   }


   public static Supplier<ControllerWalkToGoalStatusMessagePubSubType> getPubSubType()
   {
      return ControllerWalkToGoalStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerWalkToGoalStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerWalkToGoalStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_goal_x_position_, other.current_goal_x_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_goal_y_position_, other.current_goal_y_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_goal_yaw_, other.current_goal_yaw_, epsilon)) return false;

      if (!this.current_position_.epsilonEquals(other.current_position_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_reached_, other.is_reached_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerWalkToGoalStatusMessage)) return false;

      ControllerWalkToGoalStatusMessage otherMyClass = (ControllerWalkToGoalStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.current_goal_x_position_ != otherMyClass.current_goal_x_position_) return false;

      if(this.current_goal_y_position_ != otherMyClass.current_goal_y_position_) return false;

      if(this.current_goal_yaw_ != otherMyClass.current_goal_yaw_) return false;

      if (!this.current_position_.equals(otherMyClass.current_position_)) return false;
      if(this.is_reached_ != otherMyClass.is_reached_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerWalkToGoalStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("current_goal_x_position=");
      builder.append(this.current_goal_x_position_);      builder.append(", ");
      builder.append("current_goal_y_position=");
      builder.append(this.current_goal_y_position_);      builder.append(", ");
      builder.append("current_goal_yaw=");
      builder.append(this.current_goal_yaw_);      builder.append(", ");
      builder.append("current_position=");
      builder.append(this.current_position_);      builder.append(", ");
      builder.append("is_reached=");
      builder.append(this.is_reached_);
      builder.append("}");
      return builder.toString();
   }
}
