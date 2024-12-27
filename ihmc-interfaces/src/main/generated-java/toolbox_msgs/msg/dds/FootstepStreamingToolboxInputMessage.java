package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the FootstepStreamingToolbox API.
       */
public class FootstepStreamingToolboxInputMessage extends Packet<FootstepStreamingToolboxInputMessage> implements Settable<FootstepStreamingToolboxInputMessage>, EpsilonComparable<FootstepStreamingToolboxInputMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The timestamp (in nanoseconds) at which this message was generated.
            * This is used on the toolbox side to estimate things such as current tracker acceleration.
            */
   public long timestamp_;
   /**
            * Specifies which foot/tracker
            */
   public byte side_ = (byte) 255;
   /**
            * This is the current position of the robot foot frame's origin.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D robot_foot_position_in_world_;
   /**
            * This is the current orientation of the robot foot frame.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion robot_foot_orientation_in_world_;
   /**
            * This is the current position of the tracker frame's origin.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D current_position_in_world_;
   /**
            * This is the current orientation of the tracker frame.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion current_orientation_in_world_;
   /**
            * Whether the current linear velocity is defined.
            */
   public boolean has_current_velocity_;
   /**
            * The current linear velocity of the control frame's origin.
            */
   public us.ihmc.euclid.tuple3D.Vector3D current_linear_velocity_in_world_;
   /**
            * The current angular velocity of the control frame.
            */
   public us.ihmc.euclid.tuple3D.Vector3D current_angular_velocity_in_world_;

   public FootstepStreamingToolboxInputMessage()
   {
      robot_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      robot_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      current_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      current_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      current_linear_velocity_in_world_ = new us.ihmc.euclid.tuple3D.Vector3D();
      current_angular_velocity_in_world_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public FootstepStreamingToolboxInputMessage(FootstepStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      timestamp_ = other.timestamp_;

      side_ = other.side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.robot_foot_position_in_world_, robot_foot_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.robot_foot_orientation_in_world_, robot_foot_orientation_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.current_position_in_world_, current_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.current_orientation_in_world_, current_orientation_in_world_);
      has_current_velocity_ = other.has_current_velocity_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.current_linear_velocity_in_world_, current_linear_velocity_in_world_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.current_angular_velocity_in_world_, current_angular_velocity_in_world_);
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
            * The timestamp (in nanoseconds) at which this message was generated.
            * This is used on the toolbox side to estimate things such as current tracker acceleration.
            */
   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   /**
            * The timestamp (in nanoseconds) at which this message was generated.
            * This is used on the toolbox side to estimate things such as current tracker acceleration.
            */
   public long getTimestamp()
   {
      return timestamp_;
   }

   /**
            * Specifies which foot/tracker
            */
   public void setSide(byte side)
   {
      side_ = side;
   }
   /**
            * Specifies which foot/tracker
            */
   public byte getSide()
   {
      return side_;
   }


   /**
            * This is the current position of the robot foot frame's origin.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getRobotFootPositionInWorld()
   {
      return robot_foot_position_in_world_;
   }


   /**
            * This is the current orientation of the robot foot frame.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getRobotFootOrientationInWorld()
   {
      return robot_foot_orientation_in_world_;
   }


   /**
            * This is the current position of the tracker frame's origin.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getCurrentPositionInWorld()
   {
      return current_position_in_world_;
   }


   /**
            * This is the current orientation of the tracker frame.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getCurrentOrientationInWorld()
   {
      return current_orientation_in_world_;
   }

   /**
            * Whether the current linear velocity is defined.
            */
   public void setHasCurrentVelocity(boolean has_current_velocity)
   {
      has_current_velocity_ = has_current_velocity;
   }
   /**
            * Whether the current linear velocity is defined.
            */
   public boolean getHasCurrentVelocity()
   {
      return has_current_velocity_;
   }


   /**
            * The current linear velocity of the control frame's origin.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getCurrentLinearVelocityInWorld()
   {
      return current_linear_velocity_in_world_;
   }


   /**
            * The current angular velocity of the control frame.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getCurrentAngularVelocityInWorld()
   {
      return current_angular_velocity_in_world_;
   }


   public static Supplier<FootstepStreamingToolboxInputMessagePubSubType> getPubSubType()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepStreamingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.side_, other.side_, epsilon)) return false;

      if (!this.robot_foot_position_in_world_.epsilonEquals(other.robot_foot_position_in_world_, epsilon)) return false;
      if (!this.robot_foot_orientation_in_world_.epsilonEquals(other.robot_foot_orientation_in_world_, epsilon)) return false;
      if (!this.current_position_in_world_.epsilonEquals(other.current_position_in_world_, epsilon)) return false;
      if (!this.current_orientation_in_world_.epsilonEquals(other.current_orientation_in_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_current_velocity_, other.has_current_velocity_, epsilon)) return false;

      if (!this.current_linear_velocity_in_world_.epsilonEquals(other.current_linear_velocity_in_world_, epsilon)) return false;
      if (!this.current_angular_velocity_in_world_.epsilonEquals(other.current_angular_velocity_in_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepStreamingToolboxInputMessage)) return false;

      FootstepStreamingToolboxInputMessage otherMyClass = (FootstepStreamingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.side_ != otherMyClass.side_) return false;

      if (!this.robot_foot_position_in_world_.equals(otherMyClass.robot_foot_position_in_world_)) return false;
      if (!this.robot_foot_orientation_in_world_.equals(otherMyClass.robot_foot_orientation_in_world_)) return false;
      if (!this.current_position_in_world_.equals(otherMyClass.current_position_in_world_)) return false;
      if (!this.current_orientation_in_world_.equals(otherMyClass.current_orientation_in_world_)) return false;
      if(this.has_current_velocity_ != otherMyClass.has_current_velocity_) return false;

      if (!this.current_linear_velocity_in_world_.equals(otherMyClass.current_linear_velocity_in_world_)) return false;
      if (!this.current_angular_velocity_in_world_.equals(otherMyClass.current_angular_velocity_in_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("side=");
      builder.append(this.side_);      builder.append(", ");
      builder.append("robot_foot_position_in_world=");
      builder.append(this.robot_foot_position_in_world_);      builder.append(", ");
      builder.append("robot_foot_orientation_in_world=");
      builder.append(this.robot_foot_orientation_in_world_);      builder.append(", ");
      builder.append("current_position_in_world=");
      builder.append(this.current_position_in_world_);      builder.append(", ");
      builder.append("current_orientation_in_world=");
      builder.append(this.current_orientation_in_world_);      builder.append(", ");
      builder.append("has_current_velocity=");
      builder.append(this.has_current_velocity_);      builder.append(", ");
      builder.append("current_linear_velocity_in_world=");
      builder.append(this.current_linear_velocity_in_world_);      builder.append(", ");
      builder.append("current_angular_velocity_in_world=");
      builder.append(this.current_angular_velocity_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
