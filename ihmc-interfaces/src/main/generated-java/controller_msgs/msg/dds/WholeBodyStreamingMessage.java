package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * It is a lightweight version of WholeBodyTrajectoryMessage designed for streaming.
       */
public class WholeBodyStreamingMessage extends Packet<WholeBodyStreamingMessage> implements Settable<WholeBodyStreamingMessage>, EpsilonComparable<WholeBodyStreamingMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * When receiving a trajectory message that is part of a stream, the controller will extrapolate the trajectory point in the future using a simple first order integration over the given duration.
            * This integration allows to improve continuity of execution for streams.
            * If no new message is received once the integration duration has elapsed, the controller will hold the desired position and reset the desired velocity to 0.
            */
   public float stream_integration_duration_;
   /**
            * Timestamp (in nanoseconds) when this message was created.
            * The timestamp can be generated from the computer where this message originates.
            */
   public long timestamp_;
   /**
            * Information for the left hand in task-space
            */
   public boolean has_left_hand_streaming_message_;
   public ihmc_common_msgs.msg.dds.SE3StreamingMessage left_hand_streaming_message_;
   /**
            * Information for the right hand in task-space
            */
   public boolean has_right_hand_streaming_message_;
   public ihmc_common_msgs.msg.dds.SE3StreamingMessage right_hand_streaming_message_;
   /**
            * Information for the left arm joints
            */
   public boolean has_left_arm_streaming_message_;
   public controller_msgs.msg.dds.JointspaceStreamingMessage left_arm_streaming_message_;
   /**
            * Information for the right arm joints
            */
   public boolean has_right_arm_streaming_message_;
   public controller_msgs.msg.dds.JointspaceStreamingMessage right_arm_streaming_message_;
   /**
            * Information for the chest
            */
   public boolean has_chest_streaming_message_;
   public ihmc_common_msgs.msg.dds.SO3StreamingMessage chest_streaming_message_;
   /**
            * Information for the pelvis
            */
   public boolean has_pelvis_streaming_message_;
   /**
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public boolean enable_user_pelvis_control_;
   public ihmc_common_msgs.msg.dds.SE3StreamingMessage pelvis_streaming_message_;
   /**
            * Information for the neck joints
            */
   public boolean has_neck_streaming_message_;
   public controller_msgs.msg.dds.JointspaceStreamingMessage neck_streaming_message_;

   public WholeBodyStreamingMessage()
   {
      left_hand_streaming_message_ = new ihmc_common_msgs.msg.dds.SE3StreamingMessage();
      right_hand_streaming_message_ = new ihmc_common_msgs.msg.dds.SE3StreamingMessage();
      left_arm_streaming_message_ = new controller_msgs.msg.dds.JointspaceStreamingMessage();
      right_arm_streaming_message_ = new controller_msgs.msg.dds.JointspaceStreamingMessage();
      chest_streaming_message_ = new ihmc_common_msgs.msg.dds.SO3StreamingMessage();
      pelvis_streaming_message_ = new ihmc_common_msgs.msg.dds.SE3StreamingMessage();
      neck_streaming_message_ = new controller_msgs.msg.dds.JointspaceStreamingMessage();
   }

   public WholeBodyStreamingMessage(WholeBodyStreamingMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyStreamingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      stream_integration_duration_ = other.stream_integration_duration_;

      timestamp_ = other.timestamp_;

      has_left_hand_streaming_message_ = other.has_left_hand_streaming_message_;

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.staticCopy(other.left_hand_streaming_message_, left_hand_streaming_message_);
      has_right_hand_streaming_message_ = other.has_right_hand_streaming_message_;

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.staticCopy(other.right_hand_streaming_message_, right_hand_streaming_message_);
      has_left_arm_streaming_message_ = other.has_left_arm_streaming_message_;

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.staticCopy(other.left_arm_streaming_message_, left_arm_streaming_message_);
      has_right_arm_streaming_message_ = other.has_right_arm_streaming_message_;

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.staticCopy(other.right_arm_streaming_message_, right_arm_streaming_message_);
      has_chest_streaming_message_ = other.has_chest_streaming_message_;

      ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType.staticCopy(other.chest_streaming_message_, chest_streaming_message_);
      has_pelvis_streaming_message_ = other.has_pelvis_streaming_message_;

      enable_user_pelvis_control_ = other.enable_user_pelvis_control_;

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.staticCopy(other.pelvis_streaming_message_, pelvis_streaming_message_);
      has_neck_streaming_message_ = other.has_neck_streaming_message_;

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.staticCopy(other.neck_streaming_message_, neck_streaming_message_);
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
            * When receiving a trajectory message that is part of a stream, the controller will extrapolate the trajectory point in the future using a simple first order integration over the given duration.
            * This integration allows to improve continuity of execution for streams.
            * If no new message is received once the integration duration has elapsed, the controller will hold the desired position and reset the desired velocity to 0.
            */
   public void setStreamIntegrationDuration(float stream_integration_duration)
   {
      stream_integration_duration_ = stream_integration_duration;
   }
   /**
            * When receiving a trajectory message that is part of a stream, the controller will extrapolate the trajectory point in the future using a simple first order integration over the given duration.
            * This integration allows to improve continuity of execution for streams.
            * If no new message is received once the integration duration has elapsed, the controller will hold the desired position and reset the desired velocity to 0.
            */
   public float getStreamIntegrationDuration()
   {
      return stream_integration_duration_;
   }

   /**
            * Timestamp (in nanoseconds) when this message was created.
            * The timestamp can be generated from the computer where this message originates.
            */
   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   /**
            * Timestamp (in nanoseconds) when this message was created.
            * The timestamp can be generated from the computer where this message originates.
            */
   public long getTimestamp()
   {
      return timestamp_;
   }

   /**
            * Information for the left hand in task-space
            */
   public void setHasLeftHandStreamingMessage(boolean has_left_hand_streaming_message)
   {
      has_left_hand_streaming_message_ = has_left_hand_streaming_message;
   }
   /**
            * Information for the left hand in task-space
            */
   public boolean getHasLeftHandStreamingMessage()
   {
      return has_left_hand_streaming_message_;
   }


   public ihmc_common_msgs.msg.dds.SE3StreamingMessage getLeftHandStreamingMessage()
   {
      return left_hand_streaming_message_;
   }

   /**
            * Information for the right hand in task-space
            */
   public void setHasRightHandStreamingMessage(boolean has_right_hand_streaming_message)
   {
      has_right_hand_streaming_message_ = has_right_hand_streaming_message;
   }
   /**
            * Information for the right hand in task-space
            */
   public boolean getHasRightHandStreamingMessage()
   {
      return has_right_hand_streaming_message_;
   }


   public ihmc_common_msgs.msg.dds.SE3StreamingMessage getRightHandStreamingMessage()
   {
      return right_hand_streaming_message_;
   }

   /**
            * Information for the left arm joints
            */
   public void setHasLeftArmStreamingMessage(boolean has_left_arm_streaming_message)
   {
      has_left_arm_streaming_message_ = has_left_arm_streaming_message;
   }
   /**
            * Information for the left arm joints
            */
   public boolean getHasLeftArmStreamingMessage()
   {
      return has_left_arm_streaming_message_;
   }


   public controller_msgs.msg.dds.JointspaceStreamingMessage getLeftArmStreamingMessage()
   {
      return left_arm_streaming_message_;
   }

   /**
            * Information for the right arm joints
            */
   public void setHasRightArmStreamingMessage(boolean has_right_arm_streaming_message)
   {
      has_right_arm_streaming_message_ = has_right_arm_streaming_message;
   }
   /**
            * Information for the right arm joints
            */
   public boolean getHasRightArmStreamingMessage()
   {
      return has_right_arm_streaming_message_;
   }


   public controller_msgs.msg.dds.JointspaceStreamingMessage getRightArmStreamingMessage()
   {
      return right_arm_streaming_message_;
   }

   /**
            * Information for the chest
            */
   public void setHasChestStreamingMessage(boolean has_chest_streaming_message)
   {
      has_chest_streaming_message_ = has_chest_streaming_message;
   }
   /**
            * Information for the chest
            */
   public boolean getHasChestStreamingMessage()
   {
      return has_chest_streaming_message_;
   }


   public ihmc_common_msgs.msg.dds.SO3StreamingMessage getChestStreamingMessage()
   {
      return chest_streaming_message_;
   }

   /**
            * Information for the pelvis
            */
   public void setHasPelvisStreamingMessage(boolean has_pelvis_streaming_message)
   {
      has_pelvis_streaming_message_ = has_pelvis_streaming_message;
   }
   /**
            * Information for the pelvis
            */
   public boolean getHasPelvisStreamingMessage()
   {
      return has_pelvis_streaming_message_;
   }

   /**
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public void setEnableUserPelvisControl(boolean enable_user_pelvis_control)
   {
      enable_user_pelvis_control_ = enable_user_pelvis_control;
   }
   /**
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public boolean getEnableUserPelvisControl()
   {
      return enable_user_pelvis_control_;
   }


   public ihmc_common_msgs.msg.dds.SE3StreamingMessage getPelvisStreamingMessage()
   {
      return pelvis_streaming_message_;
   }

   /**
            * Information for the neck joints
            */
   public void setHasNeckStreamingMessage(boolean has_neck_streaming_message)
   {
      has_neck_streaming_message_ = has_neck_streaming_message;
   }
   /**
            * Information for the neck joints
            */
   public boolean getHasNeckStreamingMessage()
   {
      return has_neck_streaming_message_;
   }


   public controller_msgs.msg.dds.JointspaceStreamingMessage getNeckStreamingMessage()
   {
      return neck_streaming_message_;
   }


   public static Supplier<WholeBodyStreamingMessagePubSubType> getPubSubType()
   {
      return WholeBodyStreamingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyStreamingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyStreamingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stream_integration_duration_, other.stream_integration_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_left_hand_streaming_message_, other.has_left_hand_streaming_message_, epsilon)) return false;

      if (!this.left_hand_streaming_message_.epsilonEquals(other.left_hand_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_right_hand_streaming_message_, other.has_right_hand_streaming_message_, epsilon)) return false;

      if (!this.right_hand_streaming_message_.epsilonEquals(other.right_hand_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_left_arm_streaming_message_, other.has_left_arm_streaming_message_, epsilon)) return false;

      if (!this.left_arm_streaming_message_.epsilonEquals(other.left_arm_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_right_arm_streaming_message_, other.has_right_arm_streaming_message_, epsilon)) return false;

      if (!this.right_arm_streaming_message_.epsilonEquals(other.right_arm_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_chest_streaming_message_, other.has_chest_streaming_message_, epsilon)) return false;

      if (!this.chest_streaming_message_.epsilonEquals(other.chest_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_pelvis_streaming_message_, other.has_pelvis_streaming_message_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_user_pelvis_control_, other.enable_user_pelvis_control_, epsilon)) return false;

      if (!this.pelvis_streaming_message_.epsilonEquals(other.pelvis_streaming_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_neck_streaming_message_, other.has_neck_streaming_message_, epsilon)) return false;

      if (!this.neck_streaming_message_.epsilonEquals(other.neck_streaming_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyStreamingMessage)) return false;

      WholeBodyStreamingMessage otherMyClass = (WholeBodyStreamingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.stream_integration_duration_ != otherMyClass.stream_integration_duration_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.has_left_hand_streaming_message_ != otherMyClass.has_left_hand_streaming_message_) return false;

      if (!this.left_hand_streaming_message_.equals(otherMyClass.left_hand_streaming_message_)) return false;
      if(this.has_right_hand_streaming_message_ != otherMyClass.has_right_hand_streaming_message_) return false;

      if (!this.right_hand_streaming_message_.equals(otherMyClass.right_hand_streaming_message_)) return false;
      if(this.has_left_arm_streaming_message_ != otherMyClass.has_left_arm_streaming_message_) return false;

      if (!this.left_arm_streaming_message_.equals(otherMyClass.left_arm_streaming_message_)) return false;
      if(this.has_right_arm_streaming_message_ != otherMyClass.has_right_arm_streaming_message_) return false;

      if (!this.right_arm_streaming_message_.equals(otherMyClass.right_arm_streaming_message_)) return false;
      if(this.has_chest_streaming_message_ != otherMyClass.has_chest_streaming_message_) return false;

      if (!this.chest_streaming_message_.equals(otherMyClass.chest_streaming_message_)) return false;
      if(this.has_pelvis_streaming_message_ != otherMyClass.has_pelvis_streaming_message_) return false;

      if(this.enable_user_pelvis_control_ != otherMyClass.enable_user_pelvis_control_) return false;

      if (!this.pelvis_streaming_message_.equals(otherMyClass.pelvis_streaming_message_)) return false;
      if(this.has_neck_streaming_message_ != otherMyClass.has_neck_streaming_message_) return false;

      if (!this.neck_streaming_message_.equals(otherMyClass.neck_streaming_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyStreamingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("stream_integration_duration=");
      builder.append(this.stream_integration_duration_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("has_left_hand_streaming_message=");
      builder.append(this.has_left_hand_streaming_message_);      builder.append(", ");
      builder.append("left_hand_streaming_message=");
      builder.append(this.left_hand_streaming_message_);      builder.append(", ");
      builder.append("has_right_hand_streaming_message=");
      builder.append(this.has_right_hand_streaming_message_);      builder.append(", ");
      builder.append("right_hand_streaming_message=");
      builder.append(this.right_hand_streaming_message_);      builder.append(", ");
      builder.append("has_left_arm_streaming_message=");
      builder.append(this.has_left_arm_streaming_message_);      builder.append(", ");
      builder.append("left_arm_streaming_message=");
      builder.append(this.left_arm_streaming_message_);      builder.append(", ");
      builder.append("has_right_arm_streaming_message=");
      builder.append(this.has_right_arm_streaming_message_);      builder.append(", ");
      builder.append("right_arm_streaming_message=");
      builder.append(this.right_arm_streaming_message_);      builder.append(", ");
      builder.append("has_chest_streaming_message=");
      builder.append(this.has_chest_streaming_message_);      builder.append(", ");
      builder.append("chest_streaming_message=");
      builder.append(this.chest_streaming_message_);      builder.append(", ");
      builder.append("has_pelvis_streaming_message=");
      builder.append(this.has_pelvis_streaming_message_);      builder.append(", ");
      builder.append("enable_user_pelvis_control=");
      builder.append(this.enable_user_pelvis_control_);      builder.append(", ");
      builder.append("pelvis_streaming_message=");
      builder.append(this.pelvis_streaming_message_);      builder.append(", ");
      builder.append("has_neck_streaming_message=");
      builder.append(this.has_neck_streaming_message_);      builder.append(", ");
      builder.append("neck_streaming_message=");
      builder.append(this.neck_streaming_message_);
      builder.append("}");
      return builder.toString();
   }
}
