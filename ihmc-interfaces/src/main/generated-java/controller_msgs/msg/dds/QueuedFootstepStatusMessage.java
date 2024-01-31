package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message specifies the position, orientation and side (left or right) of a footstep in the queue in world frame.
       */
public class QueuedFootstepStatusMessage extends Packet<QueuedFootstepStatusMessage> implements Settable<QueuedFootstepStatusMessage>, EpsilonComparable<QueuedFootstepStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
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
   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double swing_duration_ = -1.0;
   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double transfer_duration_ = -1.0;

   public QueuedFootstepStatusMessage()
   {
      location_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public QueuedFootstepStatusMessage(QueuedFootstepStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(QueuedFootstepStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

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

   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }


   public static Supplier<QueuedFootstepStatusMessagePubSubType> getPubSubType()
   {
      return QueuedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QueuedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QueuedFootstepStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.location_.epsilonEquals(other.location_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QueuedFootstepStatusMessage)) return false;

      QueuedFootstepStatusMessage otherMyClass = (QueuedFootstepStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.location_.equals(otherMyClass.location_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QueuedFootstepStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("location=");
      builder.append(this.location_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);
      builder.append("}");
      return builder.toString();
   }
}
