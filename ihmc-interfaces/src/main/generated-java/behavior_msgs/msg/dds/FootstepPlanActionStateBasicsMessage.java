package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionStateBasicsMessage extends Packet<FootstepPlanActionStateBasicsMessage> implements Settable<FootstepPlanActionStateBasicsMessage>, EpsilonComparable<FootstepPlanActionStateBasicsMessage>
{
   /**
            * Total number of footsteps; used for walking actions
            */
   public int total_number_of_footsteps_;
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int number_of_incomplete_footsteps_;
   /**
            * Desired left footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  desired_left_footsteps_;
   /**
            * Desired right footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  desired_right_footsteps_;
   /**
            * Current left pose
            */
   public us.ihmc.euclid.geometry.Pose3D current_left_foot_pose_;
   /**
            * Current right pose
            */
   public us.ihmc.euclid.geometry.Pose3D current_right_foot_pose_;

   public FootstepPlanActionStateBasicsMessage()
   {
      desired_left_footsteps_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage> (50, new ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType());
      desired_right_footsteps_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage> (50, new ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType());
      current_left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      current_right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();

   }

   public FootstepPlanActionStateBasicsMessage(FootstepPlanActionStateBasicsMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateBasicsMessage other)
   {
      total_number_of_footsteps_ = other.total_number_of_footsteps_;

      number_of_incomplete_footsteps_ = other.number_of_incomplete_footsteps_;

      desired_left_footsteps_.set(other.desired_left_footsteps_);
      desired_right_footsteps_.set(other.desired_right_footsteps_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.current_left_foot_pose_, current_left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.current_right_foot_pose_, current_right_foot_pose_);
   }

   /**
            * Total number of footsteps; used for walking actions
            */
   public void setTotalNumberOfFootsteps(int total_number_of_footsteps)
   {
      total_number_of_footsteps_ = total_number_of_footsteps;
   }
   /**
            * Total number of footsteps; used for walking actions
            */
   public int getTotalNumberOfFootsteps()
   {
      return total_number_of_footsteps_;
   }

   /**
            * Incomplete footsteps; used for walking actions
            */
   public void setNumberOfIncompleteFootsteps(int number_of_incomplete_footsteps)
   {
      number_of_incomplete_footsteps_ = number_of_incomplete_footsteps;
   }
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int getNumberOfIncompleteFootsteps()
   {
      return number_of_incomplete_footsteps_;
   }


   /**
            * Desired left footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  getDesiredLeftFootsteps()
   {
      return desired_left_footsteps_;
   }


   /**
            * Desired right footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  getDesiredRightFootsteps()
   {
      return desired_right_footsteps_;
   }


   /**
            * Current left pose
            */
   public us.ihmc.euclid.geometry.Pose3D getCurrentLeftFootPose()
   {
      return current_left_foot_pose_;
   }


   /**
            * Current right pose
            */
   public us.ihmc.euclid.geometry.Pose3D getCurrentRightFootPose()
   {
      return current_right_foot_pose_;
   }


   public static Supplier<FootstepPlanActionStateBasicsMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionStateBasicsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionStateBasicsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionStateBasicsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_number_of_footsteps_, other.total_number_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_incomplete_footsteps_, other.number_of_incomplete_footsteps_, epsilon)) return false;

      if (this.desired_left_footsteps_.size() != other.desired_left_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.desired_left_footsteps_.size(); i++)
         {  if (!this.desired_left_footsteps_.get(i).epsilonEquals(other.desired_left_footsteps_.get(i), epsilon)) return false; }
      }

      if (this.desired_right_footsteps_.size() != other.desired_right_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.desired_right_footsteps_.size(); i++)
         {  if (!this.desired_right_footsteps_.get(i).epsilonEquals(other.desired_right_footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.current_left_foot_pose_.epsilonEquals(other.current_left_foot_pose_, epsilon)) return false;
      if (!this.current_right_foot_pose_.epsilonEquals(other.current_right_foot_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateBasicsMessage)) return false;

      FootstepPlanActionStateBasicsMessage otherMyClass = (FootstepPlanActionStateBasicsMessage) other;

      if(this.total_number_of_footsteps_ != otherMyClass.total_number_of_footsteps_) return false;

      if(this.number_of_incomplete_footsteps_ != otherMyClass.number_of_incomplete_footsteps_) return false;

      if (!this.desired_left_footsteps_.equals(otherMyClass.desired_left_footsteps_)) return false;
      if (!this.desired_right_footsteps_.equals(otherMyClass.desired_right_footsteps_)) return false;
      if (!this.current_left_foot_pose_.equals(otherMyClass.current_left_foot_pose_)) return false;
      if (!this.current_right_foot_pose_.equals(otherMyClass.current_right_foot_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateBasicsMessage {");
      builder.append("total_number_of_footsteps=");
      builder.append(this.total_number_of_footsteps_);      builder.append(", ");
      builder.append("number_of_incomplete_footsteps=");
      builder.append(this.number_of_incomplete_footsteps_);      builder.append(", ");
      builder.append("desired_left_footsteps=");
      builder.append(this.desired_left_footsteps_);      builder.append(", ");
      builder.append("desired_right_footsteps=");
      builder.append(this.desired_right_footsteps_);      builder.append(", ");
      builder.append("current_left_foot_pose=");
      builder.append(this.current_left_foot_pose_);      builder.append(", ");
      builder.append("current_right_foot_pose=");
      builder.append(this.current_right_foot_pose_);
      builder.append("}");
      return builder.toString();
   }
}
