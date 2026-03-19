package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Input message for the PDVelocityBasedGoalReacher.
       */
public class ControllerWalkToGoalMessage extends Packet<ControllerWalkToGoalMessage> implements Settable<ControllerWalkToGoalMessage>, EpsilonComparable<ControllerWalkToGoalMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Goal pose for the robot to reach.
            */
   public double x_position_;
   public double y_position_;
   public double yaw_;
   /**
            * Goal time to reach the pose. 0.0 means as quickly as possible.
            */
   public double time_to_reach_;
   /**
            * Whether to maintain this position after reaching it.
            */
   public boolean hold_position_;
   /**
            * Proximity to the goal.
            */
   public double position_proximity_ = 0.02;
   public double orientation_proximity_ = 0.1;

   public ControllerWalkToGoalMessage()
   {
   }

   public ControllerWalkToGoalMessage(ControllerWalkToGoalMessage other)
   {
      this();
      set(other);
   }

   public void set(ControllerWalkToGoalMessage other)
   {
      sequence_id_ = other.sequence_id_;

      x_position_ = other.x_position_;

      y_position_ = other.y_position_;

      yaw_ = other.yaw_;

      time_to_reach_ = other.time_to_reach_;

      hold_position_ = other.hold_position_;

      position_proximity_ = other.position_proximity_;

      orientation_proximity_ = other.orientation_proximity_;

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
            * Goal pose for the robot to reach.
            */
   public void setXPosition(double x_position)
   {
      x_position_ = x_position;
   }
   /**
            * Goal pose for the robot to reach.
            */
   public double getXPosition()
   {
      return x_position_;
   }

   public void setYPosition(double y_position)
   {
      y_position_ = y_position;
   }
   public double getYPosition()
   {
      return y_position_;
   }

   public void setYaw(double yaw)
   {
      yaw_ = yaw;
   }
   public double getYaw()
   {
      return yaw_;
   }

   /**
            * Goal time to reach the pose. 0.0 means as quickly as possible.
            */
   public void setTimeToReach(double time_to_reach)
   {
      time_to_reach_ = time_to_reach;
   }
   /**
            * Goal time to reach the pose. 0.0 means as quickly as possible.
            */
   public double getTimeToReach()
   {
      return time_to_reach_;
   }

   /**
            * Whether to maintain this position after reaching it.
            */
   public void setHoldPosition(boolean hold_position)
   {
      hold_position_ = hold_position;
   }
   /**
            * Whether to maintain this position after reaching it.
            */
   public boolean getHoldPosition()
   {
      return hold_position_;
   }

   /**
            * Proximity to the goal.
            */
   public void setPositionProximity(double position_proximity)
   {
      position_proximity_ = position_proximity;
   }
   /**
            * Proximity to the goal.
            */
   public double getPositionProximity()
   {
      return position_proximity_;
   }

   public void setOrientationProximity(double orientation_proximity)
   {
      orientation_proximity_ = orientation_proximity;
   }
   public double getOrientationProximity()
   {
      return orientation_proximity_;
   }


   public static Supplier<ControllerWalkToGoalMessagePubSubType> getPubSubType()
   {
      return ControllerWalkToGoalMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerWalkToGoalMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerWalkToGoalMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_position_, other.x_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_position_, other.y_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_, other.yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_to_reach_, other.time_to_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_position_, other.hold_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_proximity_, other.position_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_proximity_, other.orientation_proximity_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerWalkToGoalMessage)) return false;

      ControllerWalkToGoalMessage otherMyClass = (ControllerWalkToGoalMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.x_position_ != otherMyClass.x_position_) return false;

      if(this.y_position_ != otherMyClass.y_position_) return false;

      if(this.yaw_ != otherMyClass.yaw_) return false;

      if(this.time_to_reach_ != otherMyClass.time_to_reach_) return false;

      if(this.hold_position_ != otherMyClass.hold_position_) return false;

      if(this.position_proximity_ != otherMyClass.position_proximity_) return false;

      if(this.orientation_proximity_ != otherMyClass.orientation_proximity_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerWalkToGoalMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("x_position=");
      builder.append(this.x_position_);      builder.append(", ");
      builder.append("y_position=");
      builder.append(this.y_position_);      builder.append(", ");
      builder.append("yaw=");
      builder.append(this.yaw_);      builder.append(", ");
      builder.append("time_to_reach=");
      builder.append(this.time_to_reach_);      builder.append(", ");
      builder.append("hold_position=");
      builder.append(this.hold_position_);      builder.append(", ");
      builder.append("position_proximity=");
      builder.append(this.position_proximity_);      builder.append(", ");
      builder.append("orientation_proximity=");
      builder.append(this.orientation_proximity_);
      builder.append("}");
      return builder.toString();
   }
}
