package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       * It is a lightweight version of EuclideanTrajectoryMessage designed for streaming.
       */
public class EuclideanStreamingMessage extends Packet<EuclideanStreamingMessage> implements Settable<EuclideanStreamingMessage>, EpsilonComparable<EuclideanStreamingMessage>
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
            * Define the desired 3D position to be reached.
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Define the desired 3D linear velocity to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D linear_velocity_;

   public EuclideanStreamingMessage()
   {
      frame_information_ = new ihmc_common_msgs.msg.dds.FrameInformation();
      control_frame_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public EuclideanStreamingMessage(EuclideanStreamingMessage other)
   {
      this();
      set(other);
   }

   public void set(EuclideanStreamingMessage other)
   {
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.staticCopy(other.frame_information_, frame_information_);
      use_custom_control_frame_ = other.use_custom_control_frame_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.control_frame_pose_, control_frame_pose_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_velocity_, linear_velocity_);
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
            * Define the desired 3D position to be reached.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Define the desired 3D linear velocity to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getLinearVelocity()
   {
      return linear_velocity_;
   }


   public static Supplier<EuclideanStreamingMessagePubSubType> getPubSubType()
   {
      return EuclideanStreamingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EuclideanStreamingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EuclideanStreamingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.frame_information_.epsilonEquals(other.frame_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_custom_control_frame_, other.use_custom_control_frame_, epsilon)) return false;

      if (!this.control_frame_pose_.epsilonEquals(other.control_frame_pose_, epsilon)) return false;
      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.linear_velocity_.epsilonEquals(other.linear_velocity_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EuclideanStreamingMessage)) return false;

      EuclideanStreamingMessage otherMyClass = (EuclideanStreamingMessage) other;

      if (!this.frame_information_.equals(otherMyClass.frame_information_)) return false;
      if(this.use_custom_control_frame_ != otherMyClass.use_custom_control_frame_) return false;

      if (!this.control_frame_pose_.equals(otherMyClass.control_frame_pose_)) return false;
      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.linear_velocity_.equals(otherMyClass.linear_velocity_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EuclideanStreamingMessage {");
      builder.append("frame_information=");
      builder.append(this.frame_information_);      builder.append(", ");
      builder.append("use_custom_control_frame=");
      builder.append(this.use_custom_control_frame_);      builder.append(", ");
      builder.append("control_frame_pose=");
      builder.append(this.control_frame_pose_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("linear_velocity=");
      builder.append(this.linear_velocity_);
      builder.append("}");
      return builder.toString();
   }
}
