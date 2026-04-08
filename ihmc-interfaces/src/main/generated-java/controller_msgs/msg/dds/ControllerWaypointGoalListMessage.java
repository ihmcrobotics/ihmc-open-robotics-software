package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Input message to queue a sequence of goals for the PDVelocityBasedGoalReacher.
       * Each waypoint is consumed in order; the hold_position flag of the last waypoint
       * determines whether the robot holds position at the final goal.
       */
public class ControllerWaypointGoalListMessage extends Packet<ControllerWaypointGoalListMessage> implements Settable<ControllerWaypointGoalListMessage>, EpsilonComparable<ControllerWaypointGoalListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Ordered list of goals to execute in sequence.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ControllerWaypointGoalMessage>  waypoints_;

   public ControllerWaypointGoalListMessage()
   {
      waypoints_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ControllerWaypointGoalMessage> (50, new controller_msgs.msg.dds.ControllerWaypointGoalMessagePubSubType());

   }

   public ControllerWaypointGoalListMessage(ControllerWaypointGoalListMessage other)
   {
      this();
      set(other);
   }

   public void set(ControllerWaypointGoalListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      waypoints_.set(other.waypoints_);
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
            * Ordered list of goals to execute in sequence.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ControllerWaypointGoalMessage>  getWaypoints()
   {
      return waypoints_;
   }


   public static Supplier<ControllerWaypointGoalListMessagePubSubType> getPubSubType()
   {
      return ControllerWaypointGoalListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerWaypointGoalListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerWaypointGoalListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.waypoints_.size() != other.waypoints_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.waypoints_.size(); i++)
         {  if (!this.waypoints_.get(i).epsilonEquals(other.waypoints_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerWaypointGoalListMessage)) return false;

      ControllerWaypointGoalListMessage otherMyClass = (ControllerWaypointGoalListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.waypoints_.equals(otherMyClass.waypoints_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerWaypointGoalListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("waypoints=");
      builder.append(this.waypoints_);
      builder.append("}");
      return builder.toString();
   }
}
