package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the quadruped planner to execute a path composed of the given sequence of waypoints.
       * A waypoint represents a desired (x, y, yaw) position which the robot should pass through.
       * A Hermite based curve (third order) is used to interpolate the waypoints.
       */
public class QuadrupedBodyPathPlanMessage extends Packet<QuadrupedBodyPathPlanMessage> implements Settable<QuadrupedBodyPathPlanMessage>, EpsilonComparable<QuadrupedBodyPathPlanMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public boolean is_expressed_in_absolute_time_ = true;

   /**
            * List of body path waypoint
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.EuclideanTrajectoryPointMessage>  body_path_points_;

   public QuadrupedBodyPathPlanMessage()
   {



      body_path_points_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.EuclideanTrajectoryPointMessage> (50, new controller_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType());

   }

   public QuadrupedBodyPathPlanMessage(QuadrupedBodyPathPlanMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedBodyPathPlanMessage other)
   {

      sequence_id_ = other.sequence_id_;


      is_expressed_in_absolute_time_ = other.is_expressed_in_absolute_time_;


      body_path_points_.set(other.body_path_points_);
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
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public void setIsExpressedInAbsoluteTime(boolean is_expressed_in_absolute_time)
   {
      is_expressed_in_absolute_time_ = is_expressed_in_absolute_time;
   }
   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public boolean getIsExpressedInAbsoluteTime()
   {
      return is_expressed_in_absolute_time_;
   }



   /**
            * List of body path waypoint
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.EuclideanTrajectoryPointMessage>  getBodyPathPoints()
   {
      return body_path_points_;
   }


   public static Supplier<QuadrupedBodyPathPlanMessagePubSubType> getPubSubType()
   {
      return QuadrupedBodyPathPlanMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedBodyPathPlanMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyPathPlanMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_expressed_in_absolute_time_, other.is_expressed_in_absolute_time_, epsilon)) return false;


      if (this.body_path_points_.size() != other.body_path_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_points_.size(); i++)
         {  if (!this.body_path_points_.get(i).epsilonEquals(other.body_path_points_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedBodyPathPlanMessage)) return false;

      QuadrupedBodyPathPlanMessage otherMyClass = (QuadrupedBodyPathPlanMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_expressed_in_absolute_time_ != otherMyClass.is_expressed_in_absolute_time_) return false;


      if (!this.body_path_points_.equals(otherMyClass.body_path_points_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedBodyPathPlanMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_expressed_in_absolute_time=");
      builder.append(this.is_expressed_in_absolute_time_);      builder.append(", ");

      builder.append("body_path_points=");
      builder.append(this.body_path_points_);
      builder.append("}");
      return builder.toString();
   }
}
