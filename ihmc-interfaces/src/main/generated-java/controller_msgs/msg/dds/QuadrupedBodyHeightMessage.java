package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message commands the controller to move in taskspace the body to the desired height while going through the specified trajectory points.
       * Sending this command will not affect the pelvis horizontal position.
       */
public class QuadrupedBodyHeightMessage extends Packet<QuadrupedBodyHeightMessage> implements Settable<QuadrupedBodyHeightMessage>, EpsilonComparable<QuadrupedBodyHeightMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * If true, the times are expressed in absolute time relative to the robot start
            */
   public boolean is_expressed_in_absolute_time_;

   /**
            * If true, the body height is controlled, rather than the center of mass height
            */
   public boolean control_body_height_;

   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage euclidean_trajectory_;

   public QuadrupedBodyHeightMessage()
   {




      euclidean_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();

   }

   public QuadrupedBodyHeightMessage(QuadrupedBodyHeightMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedBodyHeightMessage other)
   {

      sequence_id_ = other.sequence_id_;


      is_expressed_in_absolute_time_ = other.is_expressed_in_absolute_time_;


      control_body_height_ = other.control_body_height_;


      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.euclidean_trajectory_, euclidean_trajectory_);
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
            * If true, the body height is controlled, rather than the center of mass height
            */
   public void setControlBodyHeight(boolean control_body_height)
   {
      control_body_height_ = control_body_height;
   }
   /**
            * If true, the body height is controlled, rather than the center of mass height
            */
   public boolean getControlBodyHeight()
   {
      return control_body_height_;
   }



   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclidean_trajectory_;
   }


   public static Supplier<QuadrupedBodyHeightMessagePubSubType> getPubSubType()
   {
      return QuadrupedBodyHeightMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedBodyHeightMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyHeightMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_expressed_in_absolute_time_, other.is_expressed_in_absolute_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.control_body_height_, other.control_body_height_, epsilon)) return false;


      if (!this.euclidean_trajectory_.epsilonEquals(other.euclidean_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedBodyHeightMessage)) return false;

      QuadrupedBodyHeightMessage otherMyClass = (QuadrupedBodyHeightMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_expressed_in_absolute_time_ != otherMyClass.is_expressed_in_absolute_time_) return false;


      if(this.control_body_height_ != otherMyClass.control_body_height_) return false;


      if (!this.euclidean_trajectory_.equals(otherMyClass.euclidean_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedBodyHeightMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_expressed_in_absolute_time=");
      builder.append(this.is_expressed_in_absolute_time_);      builder.append(", ");

      builder.append("control_body_height=");
      builder.append(this.control_body_height_);      builder.append(", ");

      builder.append("euclidean_trajectory=");
      builder.append(this.euclidean_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
