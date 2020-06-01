package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace the body to the desired orientation while going through the specified trajectory points.
       * A Hermite based curve (third order) is used to interpolate the orientations.
       * This message allows controlling the body orientation without interfering with position that will still be controlled to maintain the current desired capture point position.
       * To execute a normal trajectory to reach a desired body orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       */
public class QuadrupedBodyOrientationMessage extends Packet<QuadrupedBodyOrientationMessage> implements Settable<QuadrupedBodyOrientationMessage>, EpsilonComparable<QuadrupedBodyOrientationMessage>
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
            * Indicates if the given trajectory should be considered an "absolute" orientation or an "offset" orientation
            */
   public boolean is_an_offset_orientation_ = true;

   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage so3_trajectory_;

   public QuadrupedBodyOrientationMessage()
   {




      so3_trajectory_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();

   }

   public QuadrupedBodyOrientationMessage(QuadrupedBodyOrientationMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedBodyOrientationMessage other)
   {

      sequence_id_ = other.sequence_id_;


      is_expressed_in_absolute_time_ = other.is_expressed_in_absolute_time_;


      is_an_offset_orientation_ = other.is_an_offset_orientation_;


      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.so3_trajectory_, so3_trajectory_);
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
            * Indicates if the given trajectory should be considered an "absolute" orientation or an "offset" orientation
            */
   public void setIsAnOffsetOrientation(boolean is_an_offset_orientation)
   {
      is_an_offset_orientation_ = is_an_offset_orientation;
   }
   /**
            * Indicates if the given trajectory should be considered an "absolute" orientation or an "offset" orientation
            */
   public boolean getIsAnOffsetOrientation()
   {
      return is_an_offset_orientation_;
   }



   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3_trajectory_;
   }


   public static Supplier<QuadrupedBodyOrientationMessagePubSubType> getPubSubType()
   {
      return QuadrupedBodyOrientationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedBodyOrientationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyOrientationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_expressed_in_absolute_time_, other.is_expressed_in_absolute_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_an_offset_orientation_, other.is_an_offset_orientation_, epsilon)) return false;


      if (!this.so3_trajectory_.epsilonEquals(other.so3_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedBodyOrientationMessage)) return false;

      QuadrupedBodyOrientationMessage otherMyClass = (QuadrupedBodyOrientationMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_expressed_in_absolute_time_ != otherMyClass.is_expressed_in_absolute_time_) return false;


      if(this.is_an_offset_orientation_ != otherMyClass.is_an_offset_orientation_) return false;


      if (!this.so3_trajectory_.equals(otherMyClass.so3_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedBodyOrientationMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_expressed_in_absolute_time=");
      builder.append(this.is_expressed_in_absolute_time_);      builder.append(", ");

      builder.append("is_an_offset_orientation=");
      builder.append(this.is_an_offset_orientation_);      builder.append(", ");

      builder.append("so3_trajectory=");
      builder.append(this.so3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
