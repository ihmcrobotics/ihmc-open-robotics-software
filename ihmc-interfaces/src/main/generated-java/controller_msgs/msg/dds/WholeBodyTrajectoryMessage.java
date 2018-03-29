package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. Send whole body trajectories to the
 * robot. A best effort is made to execute the trajectory while balance is kept.
 */
public class WholeBodyTrajectoryMessage extends Packet<WholeBodyTrajectoryMessage>
      implements Settable<WholeBodyTrajectoryMessage>, EpsilonComparable<WholeBodyTrajectoryMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
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
    * Trajectory for the head
    */
   public controller_msgs.msg.dds.HeadTrajectoryMessage head_trajectory_message_;

   public WholeBodyTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      left_hand_trajectory_message_ = new controller_msgs.msg.dds.HandTrajectoryMessage();
      right_hand_trajectory_message_ = new controller_msgs.msg.dds.HandTrajectoryMessage();
      left_arm_trajectory_message_ = new controller_msgs.msg.dds.ArmTrajectoryMessage();
      right_arm_trajectory_message_ = new controller_msgs.msg.dds.ArmTrajectoryMessage();
      chest_trajectory_message_ = new controller_msgs.msg.dds.ChestTrajectoryMessage();
      pelvis_trajectory_message_ = new controller_msgs.msg.dds.PelvisTrajectoryMessage();
      left_foot_trajectory_message_ = new controller_msgs.msg.dds.FootTrajectoryMessage();
      right_foot_trajectory_message_ = new controller_msgs.msg.dds.FootTrajectoryMessage();
      head_trajectory_message_ = new controller_msgs.msg.dds.HeadTrajectoryMessage();
   }

   public WholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.staticCopy(other.left_hand_trajectory_message_, left_hand_trajectory_message_);
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.staticCopy(other.right_hand_trajectory_message_, right_hand_trajectory_message_);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.staticCopy(other.left_arm_trajectory_message_, left_arm_trajectory_message_);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.staticCopy(other.right_arm_trajectory_message_, right_arm_trajectory_message_);
      controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.staticCopy(other.chest_trajectory_message_, chest_trajectory_message_);
      controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.staticCopy(other.pelvis_trajectory_message_, pelvis_trajectory_message_);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.staticCopy(other.left_foot_trajectory_message_, left_foot_trajectory_message_);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.staticCopy(other.right_foot_trajectory_message_, right_foot_trajectory_message_);
      controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.staticCopy(other.head_trajectory_message_, head_trajectory_message_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
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
    * Trajectory for the head
    */
   public controller_msgs.msg.dds.HeadTrajectoryMessage getHeadTrajectoryMessage()
   {
      return head_trajectory_message_;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.left_hand_trajectory_message_.epsilonEquals(other.left_hand_trajectory_message_, epsilon))
         return false;
      if (!this.right_hand_trajectory_message_.epsilonEquals(other.right_hand_trajectory_message_, epsilon))
         return false;
      if (!this.left_arm_trajectory_message_.epsilonEquals(other.left_arm_trajectory_message_, epsilon))
         return false;
      if (!this.right_arm_trajectory_message_.epsilonEquals(other.right_arm_trajectory_message_, epsilon))
         return false;
      if (!this.chest_trajectory_message_.epsilonEquals(other.chest_trajectory_message_, epsilon))
         return false;
      if (!this.pelvis_trajectory_message_.epsilonEquals(other.pelvis_trajectory_message_, epsilon))
         return false;
      if (!this.left_foot_trajectory_message_.epsilonEquals(other.left_foot_trajectory_message_, epsilon))
         return false;
      if (!this.right_foot_trajectory_message_.epsilonEquals(other.right_foot_trajectory_message_, epsilon))
         return false;
      if (!this.head_trajectory_message_.epsilonEquals(other.head_trajectory_message_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof WholeBodyTrajectoryMessage))
         return false;

      WholeBodyTrajectoryMessage otherMyClass = (WholeBodyTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.left_hand_trajectory_message_.equals(otherMyClass.left_hand_trajectory_message_))
         return false;
      if (!this.right_hand_trajectory_message_.equals(otherMyClass.right_hand_trajectory_message_))
         return false;
      if (!this.left_arm_trajectory_message_.equals(otherMyClass.left_arm_trajectory_message_))
         return false;
      if (!this.right_arm_trajectory_message_.equals(otherMyClass.right_arm_trajectory_message_))
         return false;
      if (!this.chest_trajectory_message_.equals(otherMyClass.chest_trajectory_message_))
         return false;
      if (!this.pelvis_trajectory_message_.equals(otherMyClass.pelvis_trajectory_message_))
         return false;
      if (!this.left_foot_trajectory_message_.equals(otherMyClass.left_foot_trajectory_message_))
         return false;
      if (!this.right_foot_trajectory_message_.equals(otherMyClass.right_foot_trajectory_message_))
         return false;
      if (!this.head_trajectory_message_.equals(otherMyClass.head_trajectory_message_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("left_hand_trajectory_message=");
      builder.append(this.left_hand_trajectory_message_);
      builder.append(", ");
      builder.append("right_hand_trajectory_message=");
      builder.append(this.right_hand_trajectory_message_);
      builder.append(", ");
      builder.append("left_arm_trajectory_message=");
      builder.append(this.left_arm_trajectory_message_);
      builder.append(", ");
      builder.append("right_arm_trajectory_message=");
      builder.append(this.right_arm_trajectory_message_);
      builder.append(", ");
      builder.append("chest_trajectory_message=");
      builder.append(this.chest_trajectory_message_);
      builder.append(", ");
      builder.append("pelvis_trajectory_message=");
      builder.append(this.pelvis_trajectory_message_);
      builder.append(", ");
      builder.append("left_foot_trajectory_message=");
      builder.append(this.left_foot_trajectory_message_);
      builder.append(", ");
      builder.append("right_foot_trajectory_message=");
      builder.append(this.right_foot_trajectory_message_);
      builder.append(", ");
      builder.append("head_trajectory_message=");
      builder.append(this.head_trajectory_message_);
      builder.append("}");
      return builder.toString();
   }
}
