package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace the body to the desired pose (position & orientation) while going through the specified trajectory points.
       * A third order polynomial function is used to interpolate positions and a Hermite based curve (third order) is used to interpolate the orientations.
       * To execute a single straight line trajectory to reach a desired body pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       * Note that the body position is limited keep the robot's balance (center of mass has to remain inside the support polygon).
       */
public class QuadrupedBodyTrajectoryMessage extends Packet<QuadrupedBodyTrajectoryMessage> implements Settable<QuadrupedBodyTrajectoryMessage>, EpsilonComparable<QuadrupedBodyTrajectoryMessage>
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
            * The position/orientation trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage se3_trajectory_;

   public QuadrupedBodyTrajectoryMessage()
   {



      se3_trajectory_ = new controller_msgs.msg.dds.SE3TrajectoryMessage();

   }

   public QuadrupedBodyTrajectoryMessage(QuadrupedBodyTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedBodyTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      is_expressed_in_absolute_time_ = other.is_expressed_in_absolute_time_;


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
            * The position/orientation trajectory information.
            */
   public controller_msgs.msg.dds.SE3TrajectoryMessage getSe3Trajectory()
   {
      return se3_trajectory_;
   }


   public static Supplier<QuadrupedBodyTrajectoryMessagePubSubType> getPubSubType()
   {
      return QuadrupedBodyTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedBodyTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_expressed_in_absolute_time_, other.is_expressed_in_absolute_time_, epsilon)) return false;


      if (!this.se3_trajectory_.epsilonEquals(other.se3_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedBodyTrajectoryMessage)) return false;

      QuadrupedBodyTrajectoryMessage otherMyClass = (QuadrupedBodyTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_expressed_in_absolute_time_ != otherMyClass.is_expressed_in_absolute_time_) return false;


      if (!this.se3_trajectory_.equals(otherMyClass.se3_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedBodyTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_expressed_in_absolute_time=");
      builder.append(this.is_expressed_in_absolute_time_);      builder.append(", ");

      builder.append("se3_trajectory=");
      builder.append(this.se3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
