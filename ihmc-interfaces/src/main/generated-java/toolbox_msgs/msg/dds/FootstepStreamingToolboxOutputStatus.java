package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepStreamingToolboxOutputStatus extends Packet<FootstepStreamingToolboxOutputStatus> implements Settable<FootstepStreamingToolboxOutputStatus>, EpsilonComparable<FootstepStreamingToolboxOutputStatus>
{
   /**
          * Message used to report the computed footstep, which will be placed by the UI
          */
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies which foot
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Desired footstep position in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_foot_position_;
   /**
            * Desired footstep orientation in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion desired_foot_orientation_;

   public FootstepStreamingToolboxOutputStatus()
   {
      desired_foot_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_foot_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public FootstepStreamingToolboxOutputStatus(FootstepStreamingToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(FootstepStreamingToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_foot_position_, desired_foot_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_foot_orientation_, desired_foot_orientation_);
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
            * Specifies which foot
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Desired footstep position in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredFootPosition()
   {
      return desired_foot_position_;
   }


   /**
            * Desired footstep orientation in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getDesiredFootOrientation()
   {
      return desired_foot_orientation_;
   }


   public static Supplier<FootstepStreamingToolboxOutputStatusPubSubType> getPubSubType()
   {
      return FootstepStreamingToolboxOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepStreamingToolboxOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepStreamingToolboxOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.desired_foot_position_.epsilonEquals(other.desired_foot_position_, epsilon)) return false;
      if (!this.desired_foot_orientation_.epsilonEquals(other.desired_foot_orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepStreamingToolboxOutputStatus)) return false;

      FootstepStreamingToolboxOutputStatus otherMyClass = (FootstepStreamingToolboxOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.desired_foot_position_.equals(otherMyClass.desired_foot_position_)) return false;
      if (!this.desired_foot_orientation_.equals(otherMyClass.desired_foot_orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepStreamingToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_foot_position=");
      builder.append(this.desired_foot_position_);      builder.append(", ");
      builder.append("desired_foot_orientation=");
      builder.append(this.desired_foot_orientation_);
      builder.append("}");
      return builder.toString();
   }
}
