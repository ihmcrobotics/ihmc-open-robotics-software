package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BipedTimedStepMessage extends Packet<BipedTimedStepMessage> implements Settable<BipedTimedStepMessage>, EpsilonComparable<BipedTimedStepMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public long sequence_id_;
   public double start_time_;
   public double end_time_;
   public double swing_height_ = -1.0;
   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D location_;
   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public BipedTimedStepMessage()
   {
      location_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public BipedTimedStepMessage(BipedTimedStepMessage other)
   {
      this();
      set(other);
   }

   public void set(BipedTimedStepMessage other)
   {
      sequence_id_ = other.sequence_id_;

      start_time_ = other.start_time_;

      end_time_ = other.end_time_;

      swing_height_ = other.swing_height_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setStartTime(double start_time)
   {
      start_time_ = start_time;
   }
   public double getStartTime()
   {
      return start_time_;
   }

   public void setEndTime(double end_time)
   {
      end_time_ = end_time;
   }
   public double getEndTime()
   {
      return end_time_;
   }

   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }
   public double getSwingHeight()
   {
      return swing_height_;
   }

   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getLocation()
   {
      return location_;
   }


   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   public static Supplier<BipedTimedStepMessagePubSubType> getPubSubType()
   {
      return BipedTimedStepMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BipedTimedStepMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BipedTimedStepMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_time_, other.start_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_time_, other.end_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.location_.epsilonEquals(other.location_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BipedTimedStepMessage)) return false;

      BipedTimedStepMessage otherMyClass = (BipedTimedStepMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.start_time_ != otherMyClass.start_time_) return false;

      if(this.end_time_ != otherMyClass.end_time_) return false;

      if(this.swing_height_ != otherMyClass.swing_height_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.location_.equals(otherMyClass.location_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BipedTimedStepMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("start_time=");
      builder.append(this.start_time_);      builder.append(", ");
      builder.append("end_time=");
      builder.append(this.end_time_);      builder.append(", ");
      builder.append("swing_height=");
      builder.append(this.swing_height_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("location=");
      builder.append(this.location_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
