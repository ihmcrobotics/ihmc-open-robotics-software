package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       * It is a lightweight version of SO3TrajectoryMessage designed for streaming.
       */
public class SO3StreamingMessage extends Packet<SO3StreamingMessage> implements Settable<SO3StreamingMessage>, EpsilonComparable<SO3StreamingMessage>
{
   public ihmc_common_msgs.msg.dds.FrameInformation frame_information_;
   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public boolean use_custom_control_frame_;
   /**
            * Pose of custom control frame expressed in the end-effector frame.
            * This is the frame attached to the rigid body that the taskspace trajectory is defined for.
            */
   public us.ihmc.euclid.geometry.Pose3D control_frame_pose_;
   /**
            * Define the desired 3D orientation to be reached.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * Define the desired 3D angular velocity, expressed in world frame, to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D angular_velocity_;
   /**
            * Define the desired 3D angular acceleration, expressed in world frame,  to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D angular_acceleration_;

   public SO3StreamingMessage()
   {
      frame_information_ = new ihmc_common_msgs.msg.dds.FrameInformation();
      control_frame_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      angular_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public SO3StreamingMessage(SO3StreamingMessage other)
   {
      this();
      set(other);
   }

   public void set(SO3StreamingMessage other)
   {
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.staticCopy(other.frame_information_, frame_information_);
      use_custom_control_frame_ = other.use_custom_control_frame_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.control_frame_pose_, control_frame_pose_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_velocity_, angular_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_acceleration_, angular_acceleration_);
   }


   public ihmc_common_msgs.msg.dds.FrameInformation getFrameInformation()
   {
      return frame_information_;
   }

   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public void setUseCustomControlFrame(boolean use_custom_control_frame)
   {
      use_custom_control_frame_ = use_custom_control_frame;
   }
   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public boolean getUseCustomControlFrame()
   {
      return use_custom_control_frame_;
   }


   /**
            * Pose of custom control frame expressed in the end-effector frame.
            * This is the frame attached to the rigid body that the taskspace trajectory is defined for.
            */
   public us.ihmc.euclid.geometry.Pose3D getControlFramePose()
   {
      return control_frame_pose_;
   }


   /**
            * Define the desired 3D orientation to be reached.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   /**
            * Define the desired 3D angular velocity, expressed in world frame, to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getAngularVelocity()
   {
      return angular_velocity_;
   }


   /**
            * Define the desired 3D angular acceleration, expressed in world frame,  to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getAngularAcceleration()
   {
      return angular_acceleration_;
   }


   public static Supplier<SO3StreamingMessagePubSubType> getPubSubType()
   {
      return SO3StreamingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SO3StreamingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SO3StreamingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.frame_information_.epsilonEquals(other.frame_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_custom_control_frame_, other.use_custom_control_frame_, epsilon)) return false;

      if (!this.control_frame_pose_.epsilonEquals(other.control_frame_pose_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!this.angular_velocity_.epsilonEquals(other.angular_velocity_, epsilon)) return false;
      if (!this.angular_acceleration_.epsilonEquals(other.angular_acceleration_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SO3StreamingMessage)) return false;

      SO3StreamingMessage otherMyClass = (SO3StreamingMessage) other;

      if (!this.frame_information_.equals(otherMyClass.frame_information_)) return false;
      if(this.use_custom_control_frame_ != otherMyClass.use_custom_control_frame_) return false;

      if (!this.control_frame_pose_.equals(otherMyClass.control_frame_pose_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if (!this.angular_velocity_.equals(otherMyClass.angular_velocity_)) return false;
      if (!this.angular_acceleration_.equals(otherMyClass.angular_acceleration_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SO3StreamingMessage {");
      builder.append("frame_information=");
      builder.append(this.frame_information_);      builder.append(", ");
      builder.append("use_custom_control_frame=");
      builder.append(this.use_custom_control_frame_);      builder.append(", ");
      builder.append("control_frame_pose=");
      builder.append(this.control_frame_pose_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("angular_velocity=");
      builder.append(this.angular_velocity_);      builder.append(", ");
      builder.append("angular_acceleration=");
      builder.append(this.angular_acceleration_);
      builder.append("}");
      return builder.toString();
   }
}
