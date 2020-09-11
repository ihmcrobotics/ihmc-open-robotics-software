package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.
       */
public class WholeBodyTrajectoryMessage extends Packet<WholeBodyTrajectoryMessage> implements Settable<WholeBodyTrajectoryMessage>, EpsilonComparable<WholeBodyTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Trajectory for the left hand
            */
   public controller_msgs.msg.dds.HandTrajectoryMessage left_hand_trajectory_message_;
   /**
            * Trajectory for the right hand
            */
   public controller_msgs.msg.dds.HandTrajectoryMessage right_hand_trajectory_message_;
   /**
            * Trajectory for the left arm joints
            */
   public controller_msgs.msg.dds.ArmTrajectoryMessage left_arm_trajectory_message_;
   /**
            * Trajectory for the right arm joints
            */
   public controller_msgs.msg.dds.ArmTrajectoryMessage right_arm_trajectory_message_;
   /**
            * Trajectory for the chest
            */
   public controller_msgs.msg.dds.ChestTrajectoryMessage chest_trajectory_message_;
   /**
            * Trajectory for the spine joints
            */
   public controller_msgs.msg.dds.SpineTrajectoryMessage spine_trajectory_message_;
   /**
            * Trajectory for the pelvis
            */
   public controller_msgs.msg.dds.PelvisTrajectoryMessage pelvis_trajectory_message_;
   /**
            * Trajectory for the left foot
            */
   public controller_msgs.msg.dds.FootTrajectoryMessage left_foot_trajectory_message_;
   /**
            * Trajectory for the right foot
            */
   public controller_msgs.msg.dds.FootTrajectoryMessage right_foot_trajectory_message_;
   /**
            * Trajectory for the neck joints
            */
   public controller_msgs.msg.dds.NeckTrajectoryMessage neck_trajectory_message_;
   /**
            * Trajectory for the head
            */
   public controller_msgs.msg.dds.HeadTrajectoryMessage head_trajectory_message_;

   public WholeBodyTrajectoryMessage()
   {
      left_hand_trajectory_message_ = new controller_msgs.msg.dds.HandTrajectoryMessage();
      right_hand_trajectory_message_ = new controller_msgs.msg.dds.HandTrajectoryMessage();
      left_arm_trajectory_message_ = new controller_msgs.msg.dds.ArmTrajectoryMessage();
      right_arm_trajectory_message_ = new controller_msgs.msg.dds.ArmTrajectoryMessage();
      chest_trajectory_message_ = new controller_msgs.msg.dds.ChestTrajectoryMessage();
      spine_trajectory_message_ = new controller_msgs.msg.dds.SpineTrajectoryMessage();
      pelvis_trajectory_message_ = new controller_msgs.msg.dds.PelvisTrajectoryMessage();
      left_foot_trajectory_message_ = new controller_msgs.msg.dds.FootTrajectoryMessage();
      right_foot_trajectory_message_ = new controller_msgs.msg.dds.FootTrajectoryMessage();
      neck_trajectory_message_ = new controller_msgs.msg.dds.NeckTrajectoryMessage();
      head_trajectory_message_ = new controller_msgs.msg.dds.HeadTrajectoryMessage();
   }

   public WholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.staticCopy(other.left_hand_trajectory_message_, left_hand_trajectory_message_);
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.staticCopy(other.right_hand_trajectory_message_, right_hand_trajectory_message_);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.staticCopy(other.left_arm_trajectory_message_, left_arm_trajectory_message_);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.staticCopy(other.right_arm_trajectory_message_, right_arm_trajectory_message_);
      controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.staticCopy(other.chest_trajectory_message_, chest_trajectory_message_);
      controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType.staticCopy(other.spine_trajectory_message_, spine_trajectory_message_);
      controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.staticCopy(other.pelvis_trajectory_message_, pelvis_trajectory_message_);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.staticCopy(other.left_foot_trajectory_message_, left_foot_trajectory_message_);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.staticCopy(other.right_foot_trajectory_message_, right_foot_trajectory_message_);
      controller_msgs.msg.dds.NeckTrajectoryMessagePubSubType.staticCopy(other.neck_trajectory_message_, neck_trajectory_message_);
      controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.staticCopy(other.head_trajectory_message_, head_trajectory_message_);
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
            * Trajectory for the left hand
            */
   public controller_msgs.msg.dds.HandTrajectoryMessage getLeftHandTrajectoryMessage()
   {
      return left_hand_trajectory_message_;
   }


   /**
            * Trajectory for the right hand
            */
   public controller_msgs.msg.dds.HandTrajectoryMessage getRightHandTrajectoryMessage()
   {
      return right_hand_trajectory_message_;
   }


   /**
            * Trajectory for the left arm joints
            */
   public controller_msgs.msg.dds.ArmTrajectoryMessage getLeftArmTrajectoryMessage()
   {
      return left_arm_trajectory_message_;
   }


   /**
            * Trajectory for the right arm joints
            */
   public controller_msgs.msg.dds.ArmTrajectoryMessage getRightArmTrajectoryMessage()
   {
      return right_arm_trajectory_message_;
   }


   /**
            * Trajectory for the chest
            */
   public controller_msgs.msg.dds.ChestTrajectoryMessage getChestTrajectoryMessage()
   {
      return chest_trajectory_message_;
   }


   /**
            * Trajectory for the spine joints
            */
   public controller_msgs.msg.dds.SpineTrajectoryMessage getSpineTrajectoryMessage()
   {
      return spine_trajectory_message_;
   }


   /**
            * Trajectory for the pelvis
            */
   public controller_msgs.msg.dds.PelvisTrajectoryMessage getPelvisTrajectoryMessage()
   {
      return pelvis_trajectory_message_;
   }


   /**
            * Trajectory for the left foot
            */
   public controller_msgs.msg.dds.FootTrajectoryMessage getLeftFootTrajectoryMessage()
   {
      return left_foot_trajectory_message_;
   }


   /**
            * Trajectory for the right foot
            */
   public controller_msgs.msg.dds.FootTrajectoryMessage getRightFootTrajectoryMessage()
   {
      return right_foot_trajectory_message_;
   }


   /**
            * Trajectory for the neck joints
            */
   public controller_msgs.msg.dds.NeckTrajectoryMessage getNeckTrajectoryMessage()
   {
      return neck_trajectory_message_;
   }


   /**
            * Trajectory for the head
            */
   public controller_msgs.msg.dds.HeadTrajectoryMessage getHeadTrajectoryMessage()
   {
      return head_trajectory_message_;
   }


   public static Supplier<WholeBodyTrajectoryMessagePubSubType> getPubSubType()
   {
      return WholeBodyTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.left_hand_trajectory_message_.epsilonEquals(other.left_hand_trajectory_message_, epsilon)) return false;
      if (!this.right_hand_trajectory_message_.epsilonEquals(other.right_hand_trajectory_message_, epsilon)) return false;
      if (!this.left_arm_trajectory_message_.epsilonEquals(other.left_arm_trajectory_message_, epsilon)) return false;
      if (!this.right_arm_trajectory_message_.epsilonEquals(other.right_arm_trajectory_message_, epsilon)) return false;
      if (!this.chest_trajectory_message_.epsilonEquals(other.chest_trajectory_message_, epsilon)) return false;
      if (!this.spine_trajectory_message_.epsilonEquals(other.spine_trajectory_message_, epsilon)) return false;
      if (!this.pelvis_trajectory_message_.epsilonEquals(other.pelvis_trajectory_message_, epsilon)) return false;
      if (!this.left_foot_trajectory_message_.epsilonEquals(other.left_foot_trajectory_message_, epsilon)) return false;
      if (!this.right_foot_trajectory_message_.epsilonEquals(other.right_foot_trajectory_message_, epsilon)) return false;
      if (!this.neck_trajectory_message_.epsilonEquals(other.neck_trajectory_message_, epsilon)) return false;
      if (!this.head_trajectory_message_.epsilonEquals(other.head_trajectory_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyTrajectoryMessage)) return false;

      WholeBodyTrajectoryMessage otherMyClass = (WholeBodyTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.left_hand_trajectory_message_.equals(otherMyClass.left_hand_trajectory_message_)) return false;
      if (!this.right_hand_trajectory_message_.equals(otherMyClass.right_hand_trajectory_message_)) return false;
      if (!this.left_arm_trajectory_message_.equals(otherMyClass.left_arm_trajectory_message_)) return false;
      if (!this.right_arm_trajectory_message_.equals(otherMyClass.right_arm_trajectory_message_)) return false;
      if (!this.chest_trajectory_message_.equals(otherMyClass.chest_trajectory_message_)) return false;
      if (!this.spine_trajectory_message_.equals(otherMyClass.spine_trajectory_message_)) return false;
      if (!this.pelvis_trajectory_message_.equals(otherMyClass.pelvis_trajectory_message_)) return false;
      if (!this.left_foot_trajectory_message_.equals(otherMyClass.left_foot_trajectory_message_)) return false;
      if (!this.right_foot_trajectory_message_.equals(otherMyClass.right_foot_trajectory_message_)) return false;
      if (!this.neck_trajectory_message_.equals(otherMyClass.neck_trajectory_message_)) return false;
      if (!this.head_trajectory_message_.equals(otherMyClass.head_trajectory_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("left_hand_trajectory_message=");
      builder.append(this.left_hand_trajectory_message_);      builder.append(", ");
      builder.append("right_hand_trajectory_message=");
      builder.append(this.right_hand_trajectory_message_);      builder.append(", ");
      builder.append("left_arm_trajectory_message=");
      builder.append(this.left_arm_trajectory_message_);      builder.append(", ");
      builder.append("right_arm_trajectory_message=");
      builder.append(this.right_arm_trajectory_message_);      builder.append(", ");
      builder.append("chest_trajectory_message=");
      builder.append(this.chest_trajectory_message_);      builder.append(", ");
      builder.append("spine_trajectory_message=");
      builder.append(this.spine_trajectory_message_);      builder.append(", ");
      builder.append("pelvis_trajectory_message=");
      builder.append(this.pelvis_trajectory_message_);      builder.append(", ");
      builder.append("left_foot_trajectory_message=");
      builder.append(this.left_foot_trajectory_message_);      builder.append(", ");
      builder.append("right_foot_trajectory_message=");
      builder.append(this.right_foot_trajectory_message_);      builder.append(", ");
      builder.append("neck_trajectory_message=");
      builder.append(this.neck_trajectory_message_);      builder.append(", ");
      builder.append("head_trajectory_message=");
      builder.append(this.head_trajectory_message_);
      builder.append("}");
      return builder.toString();
   }
}
